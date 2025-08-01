# Gazebo Deployment Guide - Reusability & Maintainability

## 🎯 **Best Gazebo Options for Reusability and Maintainability**

### **🏆 RECOMMENDED APPROACH: Multi-Platform Docker Configuration**

This guide provides the optimal Gazebo deployment strategy that maximizes reusability and maintainability across different platforms.

## **📋 Configuration Options**

### **1. 🍎 Apple Silicon (ARM64) - RECOMMENDED**
```bash
# Automatic deployment
python3 scripts/deploy_gazebo.py

# Manual deployment
docker-compose --env-file config/apple_silicon.env up -d
```

**Advantages:**
- ✅ Native ARM64 performance
- ✅ No emulation overhead
- ✅ Fastest response times
- ✅ Best resource utilization
- ✅ Future-proof for Apple Silicon

**Configuration:**
- Image: `arm64v8/gazebo:gzserver11-focal`
- Platform: `linux/arm64`
- Memory: 4GB (optimized)
- CPUs: 2 cores

### **2. 🖥️ AMD64 with Emulation - FALLBACK**
```bash
# Manual deployment
docker-compose --env-file config/amd64.env up -d
```

**Advantages:**
- ✅ Universal compatibility
- ✅ Works on all platforms
- ✅ Stable and tested
- ✅ Good performance with emulation

**Configuration:**
- Image: `gazebo:gzserver11-focal`
- Platform: `linux/amd64`
- Memory: 6GB (emulation overhead)
- CPUs: 3 cores

### **3. 🖥️ Headless Mode - SERVER DEPLOYMENT**
```bash
# Headless deployment
python3 scripts/deploy_gazebo.py --headless
```

**Advantages:**
- ✅ Minimal resource usage
- ✅ Perfect for CI/CD
- ✅ Server-friendly
- ✅ No GUI dependencies

**Configuration:**
- Image: `arm64v8/gazebo:gzserver11-focal`
- Platform: `linux/arm64`
- Memory: 3GB (reduced)
- CPUs: 2 cores
- Headless: true

## **🚀 Deployment Strategy**

### **Automatic Deployment (Recommended)**
```bash
# Deploy with automatic platform detection
python3 scripts/deploy_gazebo.py
```

**Features:**
- 🔍 Automatic platform detection
- 🧪 Image availability testing
- 📊 Resource requirement checking
- ✅ Health monitoring
- 🔄 Automatic fallback

### **Manual Deployment**
```bash
# Apple Silicon
docker-compose --env-file config/apple_silicon.env up -d

# AMD64
docker-compose --env-file config/amd64.env up -d

# Headless
docker-compose --env-file config/headless.env up -d
```

## **🔧 Configuration Files**

### **Environment Variables**
Each configuration file contains optimized settings:

```bash
# Apple Silicon (config/apple_silicon.env)
GAZEBO_IMAGE=arm64v8/gazebo:gzserver11-focal
GAZEBO_PLATFORM=linux/arm64
GAZEBO_MEMORY_LIMIT=4G
GAZEBO_CPU_LIMIT=2.0

# AMD64 (config/amd64.env)
GAZEBO_IMAGE=gazebo:gzserver11-focal
GAZEBO_PLATFORM=linux/amd64
GAZEBO_MEMORY_LIMIT=6G
GAZEBO_CPU_LIMIT=3.0

# Headless (config/headless.env)
GAZEBO_IMAGE=arm64v8/gazebo:gzserver11-focal
GAZEBO_PLATFORM=linux/arm64
GAZEBO_USE_HEADLESS=true
GAZEBO_MEMORY_LIMIT=3G
```

## **📊 Resource Requirements**

### **Minimum Requirements**
- **Memory:** 8GB Docker allocation
- **CPUs:** 4 cores minimum
- **Storage:** 64GB available space
- **Network:** Stable internet connection

### **Recommended Requirements**
- **Memory:** 12GB Docker allocation
- **CPUs:** 6+ cores
- **Storage:** 128GB available space
- **Network:** High-speed connection

## **🔄 Maintenance Features**

### **Health Monitoring**
```yaml
healthcheck:
  test: ["CMD", "curl", "-f", "http://localhost:11345"]
  interval: 30s
  timeout: 10s
  retries: 3
  start_period: 40s
```

### **Resource Limits**
```yaml
deploy:
  resources:
    limits:
      memory: 4G
      cpus: '2.0'
    reservations:
      memory: 2G
      cpus: '1.0'
```

### **Automatic Restart**
```yaml
restart: unless-stopped
```

## **🎮 Usage Commands**

### **Start Services**
```bash
# Automatic deployment
python3 scripts/deploy_gazebo.py

# Manual deployment
docker-compose --env-file config/apple_silicon.env up -d
```

### **Monitor Services**
```bash
# View logs
docker-compose logs -f

# Check status
docker-compose ps

# Health check
docker-compose exec gazebo curl -f http://localhost:11345
```

### **Stop Services**
```bash
# Stop all services
docker-compose down

# Stop specific service
docker-compose stop gazebo
```

### **Update Services**
```bash
# Pull latest images
docker-compose pull

# Rebuild and restart
docker-compose up -d --build
```

## **🔍 Troubleshooting**

### **Common Issues**

**1. Platform Compatibility**
```bash
# Check platform
uname -m

# Force platform
docker run --platform linux/amd64 gazebo:gzserver11-focal
```

**2. Resource Issues**
```bash
# Check Docker resources
docker system info

# Increase memory in Docker Desktop
# Settings > Resources > Memory: 12GB
```

**3. Network Issues**
```bash
# Check ports
lsof -i :11345

# Restart Docker
docker system restart
```

### **Performance Optimization**

**1. Apple Silicon Optimization**
- Use native ARM64 images
- Allocate adequate memory (12GB+)
- Enable virtualization framework

**2. AMD64 Optimization**
- Use platform emulation
- Increase memory allocation
- Monitor CPU usage

**3. Headless Optimization**
- Disable GUI components
- Reduce memory allocation
- Use server-optimized images

## **📈 Performance Comparison**

| Configuration | Memory | CPU | Response Time | Compatibility |
|---------------|--------|-----|---------------|---------------|
| Apple Silicon | 4GB | 2 cores | ⚡ Fast | 🍎 Native |
| AMD64 Emulation | 6GB | 3 cores | 🐌 Slower | 🌍 Universal |
| Headless | 3GB | 2 cores | ⚡ Fast | 🖥️ Server |

## **🎯 Best Practices**

### **For Development**
1. Use Apple Silicon configuration on M1/M2 Macs
2. Enable health monitoring
3. Use volume mounts for code changes
4. Monitor resource usage

### **For Production**
1. Use headless configuration
2. Implement proper logging
3. Set resource limits
4. Use restart policies

### **For CI/CD**
1. Use headless mode
2. Minimize resource usage
3. Implement health checks
4. Use deterministic configurations

## **🚀 Quick Start**

```bash
# 1. Configure Docker resources (12GB memory)
# 2. Run automatic deployment
python3 scripts/deploy_gazebo.py

# 3. Verify deployment
docker-compose ps

# 4. Access services
# Gazebo: http://localhost:11345
# Control Panel: http://localhost:8080
# AI Service: http://localhost:5000
```

## **📚 Additional Resources**

- [Gazebo Documentation](http://gazebosim.org/docs)
- [Docker Compose Reference](https://docs.docker.com/compose/)
- [Apple Silicon Docker Guide](https://docs.docker.com/desktop/mac/apple-silicon/)

---

**🎉 This configuration provides maximum reusability and maintainability across all platforms!** 