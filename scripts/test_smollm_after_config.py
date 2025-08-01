#!/usr/bin/env python3
"""
Test SMOLLM after Docker resource configuration
"""

import json
import urllib.request
import time
from datetime import datetime

def log(message, level="INFO"):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] {message}")

def test_smollm():
    """Test SMOLLM with proper resources"""
    log("🧪 Testing SMOLLM with configured resources...")
    
    try:
        # Test 1: Check if Ollama is running
        log("🔍 Checking Ollama service...")
        response = urllib.request.urlopen("http://localhost:11434/api/tags", timeout=10)
        data = json.loads(response.read().decode('utf-8'))
        
        models = [model['name'] for model in data.get('models', [])]
        if 'smollm:135m' not in models:
            log("❌ SMOLLM model not available", "ERROR")
            return False
        
        log("✅ SMOLLM model available", "SUCCESS")
        
        # Test 2: Simple inference test
        log("🚀 Testing SMOLLM inference...")
        
        test_data = {
            "model": "smollm:135m",
            "messages": [{"role": "user", "content": "Say 'Hello from SMOLLM'"}],
            "stream": False,
            "options": {
                "temperature": 0,
                "num_predict": 10,
                "top_k": 1,
                "top_p": 0.1,
            }
        }
        
        req = urllib.request.Request(
            "http://localhost:11434/api/chat",
            data=json.dumps(test_data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        
        start_time = time.time()
        response = urllib.request.urlopen(req, timeout=30)
        end_time = time.time()
        
        result = json.loads(response.read().decode('utf-8'))
        response_time = end_time - start_time
        
        if 'message' in result and 'content' in result['message']:
            log(f"✅ SMOLLM test successful! Response: '{result['message']['content']}'", "SUCCESS")
            log(f"⏱️ Response time: {response_time:.2f} seconds")
            
            if response_time < 5.0:
                log("⚡ Fast response - resources are well configured!", "SUCCESS")
            elif response_time < 10.0:
                log("✅ Good response time", "SUCCESS")
            else:
                log("⚠️ Slow response - consider allocating more resources", "WARNING")
            
            return True
        else:
            log("❌ SMOLLM test failed - no valid response", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ SMOLLM test failed: {e}", "ERROR")
        return False

def test_ai_service():
    """Test AI service with SMOLLM"""
    log("🤖 Testing AI service with SMOLLM...")
    
    try:
        # Start AI service
        import subprocess
        log("🚀 Starting AI service...")
        
        subprocess.run([
            "docker", "run", "-d", "--name", "test_ai_smollm", 
            "-p", "5003:5000", "paris_swarm_ai"
        ], check=True)
        
        time.sleep(10)
        
        # Test health
        response = urllib.request.urlopen("http://localhost:5003/health", timeout=10)
        health_data = json.loads(response.read().decode('utf-8'))
        
        if health_data.get('status') == 'healthy':
            log("✅ AI service health check passed", "SUCCESS")
            
            # Test flight decision
            flight_data = {
                "scenario": "Quadcopter at position [10, 5, 15], obstacle detected ahead, battery 85%",
                "quad_id": "quad_001"
            }
            
            req = urllib.request.Request(
                "http://localhost:5003/flight_decision",
                data=json.dumps(flight_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            
            start_time = time.time()
            response = urllib.request.urlopen(req, timeout=15)
            end_time = time.time()
            
            decision_data = json.loads(response.read().decode('utf-8'))
            response_time = end_time - start_time
            
            if 'decision' in decision_data:
                log(f"✅ AI flight decision: {decision_data['decision']}", "SUCCESS")
                log(f"⏱️ AI response time: {response_time:.2f} seconds")
                
                if response_time < 5.0:
                    log("⚡ Fast AI response - SMOLLM working well!", "SUCCESS")
                else:
                    log("✅ AI response received", "SUCCESS")
                
                return True
            else:
                log("❌ AI flight decision failed", "ERROR")
                return False
        else:
            log("❌ AI service health check failed", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ AI service test failed: {e}", "ERROR")
        return False
    finally:
        # Cleanup
        try:
            subprocess.run(["docker", "rm", "-f", "test_ai_smollm"], check=False)
        except:
            pass

def main():
    """Main test function"""
    print("🚁 PARIS SWARM - SMOLLM CONFIGURATION TEST")
    print("="*50)
    
    # Test SMOLLM directly
    smollm_ok = test_smollm()
    
    if smollm_ok:
        # Test AI service with SMOLLM
        ai_ok = test_ai_service()
        
        if ai_ok:
            print("\n🎉 SMOLLM is working perfectly!")
            print("🚀 You can now run the full simulation with AI support.")
            print("💡 The AI will make intelligent flight decisions in real-time.")
        else:
            print("\n⚠️ SMOLLM works but AI service needs attention.")
    else:
        print("\n❌ SMOLLM is not working properly.")
        print("🔧 Please check Docker resource configuration.")
        print("📋 Make sure you allocated at least 8GB memory to Docker.")

if __name__ == "__main__":
    main() 