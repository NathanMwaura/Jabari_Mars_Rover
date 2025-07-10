class SystemMonitor {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.isConnected = false;
        this.connectionStatus = 'disconnected';
        this.systemStats = {
            uptime: 0,
            networkLatency: 0,
            dataTransfer: { sent: 0, received: 0 },
            errors: 0,
            warnings: 0,
            cpuUsage: 0,
            memoryUsage: 0,
            diskUsage: 0,
            temperature: 0
        };
        
        this.logHistory = [];
        this.maxLogEntries = 100;
        this.updateInterval = null;
        this.startTime = Date.now();
        
        this.init();
    }

    init() {
        this.createInterface();
        this.startMonitoring();
        this.bindEvents();
        this.simulateSystemData();
    }

    createInterface() {
        this.container.innerHTML = `
            <div class="system-monitor-container">
                <div class="monitor-header">
                    <h3>System Monitor</h3>
                    <div class="connection-status ${this.connectionStatus}">
                        <span class="status-indicator"></span>
                        <span class="status-text">Demo Mode</span>
                    </div>
                </div>
                
                <div class="system-stats-grid">
                    <div class="stat-card">
                        <div class="stat-icon">⏱️</div>
                        <div class="stat-info">
                            <div class="stat-label">System Uptime</div>
                            <div class="stat-value" id="uptimeValue">00:00:00</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">📡</div>
                        <div class="stat-info">
                            <div class="stat-label">Network Latency</div>
                            <div class="stat-value" id="latencyValue">-- ms</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">🔄</div>
                        <div class="stat-info">
                            <div class="stat-label">Data Transfer</div>
                            <div class="stat-value" id="dataTransferValue">0 KB</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">⚠️</div>
                        <div class="stat-info">
                            <div class="stat-label">System Alerts</div>
                            <div class="stat-value" id="alertsValue">0</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">🖥️</div>
                        <div class="stat-info">
                            <div class="stat-label">CPU Usage</div>
                            <div class="stat-value" id="cpuValue">0%</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">💾</div>
                        <div class="stat-info">
                            <div class="stat-label">Memory Usage</div>
                            <div class="stat-value" id="memoryValue">0%</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">🌡️</div>
                        <div class="stat-info">
                            <div class="stat-label">Temperature</div>
                            <div class="stat-value" id="temperatureValue">25°C</div>
                        </div>
                    </div>
                    
                    <div class="stat-card">
                        <div class="stat-icon">💽</div>
                        <div class="stat-info">
                            <div class="stat-label">Disk Usage</div>
                            <div class="stat-value" id="diskValue">0%</div>
                        </div>
                    </div>
                </div>

                <div class="system-charts">
                    <div class="chart-container">
                        <h4>System Performance</h4>
                        <canvas id="performanceChart" width="400" height="200"></canvas>
                    </div>
                    
                    <div class="chart-container">
                        <h4>Network Activity</h4>
                        <canvas id="networkChart" width="400" height="200"></canvas>
                    </div>
                </div>

                <div class="system-logs">
                    <div class="logs-header">
                        <h4>System Logs</h4>
                        <div class="log-controls">
                            <button id="clearLogsBtn" class="btn btn-secondary">Clear Logs</button>
                            <button id="exportLogsBtn" class="btn btn-secondary">Export</button>
                        </div>
                    </div>
                    <div class="logs-container" id="logsContainer">
                        <!-- Logs will be populated here -->
                    </div>
                </div>
            </div>
        `;
    }

    startMonitoring() {
        this.updateInterval = setInterval(() => {
            this.updateSystemStats();
            this.updateCharts();
        }, 1000);
    }

    stopMonitoring() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }

    updateSystemStats() {
        // Update uptime
        const uptime = Date.now() - this.startTime;
        document.getElementById('uptimeValue').textContent = this.formatUptime(uptime);

        // Update other stats
        document.getElementById('latencyValue').textContent = `${this.systemStats.networkLatency} ms`;
        document.getElementById('dataTransferValue').textContent = 
            `${(this.systemStats.dataTransfer.sent / 1024).toFixed(1)} KB`;
        document.getElementById('alertsValue').textContent = 
            `${this.systemStats.errors + this.systemStats.warnings}`;
        document.getElementById('cpuValue').textContent = `${this.systemStats.cpuUsage}%`;
        document.getElementById('memoryValue').textContent = `${this.systemStats.memoryUsage}%`;
        document.getElementById('temperatureValue').textContent = `${this.systemStats.temperature}°C`;
        document.getElementById('diskValue').textContent = `${this.systemStats.diskUsage}%`;
    }

    simulateSystemData() {
        setInterval(() => {
            // Simulate network latency
            this.systemStats.networkLatency = Math.floor(Math.random() * 50) + 20;
            
            // Simulate data transfer
            this.systemStats.dataTransfer.sent += Math.floor(Math.random() * 1000) + 500;
            this.systemStats.dataTransfer.received += Math.floor(Math.random() * 1500) + 800;
            
            // Simulate CPU usage
            this.systemStats.cpuUsage = Math.floor(Math.random() * 30) + 15;
            
            // Simulate memory usage
            this.systemStats.memoryUsage = Math.floor(Math.random() * 40) + 30;
            
            // Simulate disk usage
            this.systemStats.diskUsage = Math.floor(Math.random() * 10) + 45;
            
            // Simulate temperature
            this.systemStats.temperature = Math.floor(Math.random() * 15) + 25;
            
            // Occasionally add system logs
            if (Math.random() < 0.3) {
                this.addSystemLog();
            }
        }, 2000);
    }

    addSystemLog() {
        const logTypes = ['info', 'warning', 'error', 'success'];
        const logMessages = [
            'System health check completed',
            'Network connection established',
            'Camera feed initialized',
            'Sensor data received',
            'Motor control response timeout',
            'Battery level: 85%',
            'Temperature within normal range',
            'Robot position updated',
            'Command executed successfully',
            'Wireless signal strength: Good'
        ];

        const type = logTypes[Math.floor(Math.random() * logTypes.length)];
        const message = logMessages[Math.floor(Math.random() * logMessages.length)];
        
        const logEntry = {
            timestamp: new Date(),
            type: type,
            message: message
        };

        this.logHistory.unshift(logEntry);
        
        // Keep only the last maxLogEntries entries
        if (this.logHistory.length > this.maxLogEntries) {
            this.logHistory.pop();
        }

        // Update log display
        this.updateLogDisplay();
        
        // Update error/warning counts
        if (type === 'error') {
            this.systemStats.errors++;
        } else if (type === 'warning') {
            this.systemStats.warnings++;
        }
    }

    updateLogDisplay() {
        const logsContainer = document.getElementById('logsContainer');
        logsContainer.innerHTML = this.logHistory.map(log => `
            <div class="log-entry log-${log.type}">
                <span class="log-timestamp">${log.timestamp.toLocaleTimeString()}</span>
                <span class="log-type">[${log.type.toUpperCase()}]</span>
                <span class="log-message">${log.message}</span>
            </div>
        `).join('');
    }

    updateCharts() {
        this.updatePerformanceChart();
        this.updateNetworkChart();
    }

    updatePerformanceChart() {
        const canvas = document.getElementById('performanceChart');
        const ctx = canvas.getContext('2d');
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw simple performance chart
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        const points = 20;
        for (let i = 0; i < points; i++) {
            const x = (canvas.width / points) * i;
            const y = canvas.height - (this.systemStats.cpuUsage / 100) * canvas.height + 
                     Math.sin(i * 0.5) * 20;
            
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        
        ctx.stroke();
    }

    updateNetworkChart() {
        const canvas = document.getElementById('networkChart');
        const ctx = canvas.getContext('2d');
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw network activity bars
        ctx.fillStyle = '#0066cc';
        const barWidth = canvas.width / 10;
        
        for (let i = 0; i < 10; i++) {
            const height = Math.random() * canvas.height * 0.8;
            const x = i * barWidth;
            const y = canvas.height - height;
            
            ctx.fillRect(x, y, barWidth - 2, height);
        }
    }

    bindEvents() {
        // Clear logs button
        document.getElementById('clearLogsBtn').addEventListener('click', () => {
            this.logHistory = [];
            this.updateLogDisplay();
            this.systemStats.errors = 0;
            this.systemStats.warnings = 0;
        });

        // Export logs button
        document.getElementById('exportLogsBtn').addEventListener('click', () => {
            this.exportLogs();
        });
    }

    exportLogs() {
        const logData = this.logHistory.map(log => 
            `${log.timestamp.toISOString()} [${log.type.toUpperCase()}] ${log.message}`
        ).join('\n');
        
        const blob = new Blob([logData], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `system-logs-${new Date().toISOString().split('T')[0]}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    formatUptime(milliseconds) {
        const seconds = Math.floor(milliseconds / 1000);
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const secs = seconds % 60;
        
        return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
    }

    // Public methods for external control
    connect() {
        this.isConnected = true;
        this.connectionStatus = 'connected';
        this.updateConnectionStatus();
    }

    disconnect() {
        this.isConnected = false;
        this.connectionStatus = 'disconnected';
        this.updateConnectionStatus();
    }

    updateConnectionStatus() {
        const statusElement = document.querySelector('.connection-status');
        const statusText = document.querySelector('.status-text');
        
        statusElement.className = `connection-status ${this.connectionStatus}`;
        statusText.textContent = this.isConnected ? 'Connected' : 'Demo Mode';
    }

    // Cleanup method
    destroy() {
        this.stopMonitoring();
        if (this.container) {
            this.container.innerHTML = '';
        }
    }
}