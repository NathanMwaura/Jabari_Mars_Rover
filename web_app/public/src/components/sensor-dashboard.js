class SensorDashboard {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.isActive = false;
        this.updateInterval = null;
        this.chartUpdateInterval = null;
        
        // Sensor data storage
        this.sensorData = {
            temperature: [],
            humidity: [],
            battery: [],
            current: [],
            voltage: [],
            cpu: [],
            memory: [],
            timestamps: []
        };
        
        // Chart instances
        this.charts = {};
        this.currentChart = 'temperature';
        
        // Configuration
        this.maxDataPoints = 50;
        this.updateFrequency = 1000; // ms
        
        // Alert history
        this.alertHistory = [];
        this.maxAlerts = 10;
        
        this.init();
    }

    init() {
        this.createDashboardInterface();
        this.initializeCharts();
        this.startDataSimulation();
        this.bindEvents();
        this.isActive = true;
    }

    createDashboardInterface() {
        this.container.innerHTML = `
            <div class="sensor-dashboard-container">
                <div class="dashboard-header">
                    <h3>Sensor Dashboard</h3>
                    <div class="dashboard-controls">
                        <button id="pauseBtn" class="control-btn">
                            <span class="btn-icon">⏸</span>
                            <span class="btn-text">Pause</span>
                        </button>
                        <button id="resetBtn" class="control-btn">
                            <span class="btn-icon">🔄</span>
                            <span class="btn-text">Reset</span>
                        </button>
                        <button id="exportBtn" class="control-btn">
                            <span class="btn-icon">💾</span>
                            <span class="btn-text">Export</span>
                        </button>
                    </div>
                </div>

                <div class="sensor-grid">
                    <!-- Environmental Sensors -->
                    <div class="sensor-section">
                        <h4>Environmental</h4>
                        <div class="sensor-cards">
                            <div class="sensor-card temperature">
                                <div class="sensor-icon">🌡️</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Temperature</div>
                                    <div class="sensor-value" id="tempValue">--°C</div>
                                    <div class="sensor-status" id="tempStatus">Normal</div>
                                </div>
                            </div>
                            <div class="sensor-card humidity">
                                <div class="sensor-icon">💧</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Humidity</div>
                                    <div class="sensor-value" id="humidityValue">--%</div>
                                    <div class="sensor-status" id="humidityStatus">Normal</div>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- Power System -->
                    <div class="sensor-section">
                        <h4>Power System</h4>
                        <div class="sensor-cards">
                            <div class="sensor-card battery">
                                <div class="sensor-icon">🔋</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Battery</div>
                                    <div class="sensor-value" id="batteryValue">--%</div>
                                    <div class="sensor-status" id="batteryStatus">Good</div>
                                </div>
                                <div class="battery-indicator">
                                    <div class="battery-level" id="batteryLevel"></div>
                                </div>
                            </div>
                            <div class="sensor-card current">
                                <div class="sensor-icon">⚡</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Current</div>
                                    <div class="sensor-value" id="currentValue">--A</div>
                                    <div class="sensor-status" id="currentStatus">Normal</div>
                                </div>
                            </div>
                            <div class="sensor-card voltage">
                                <div class="sensor-icon">🔌</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Voltage</div>
                                    <div class="sensor-value" id="voltageValue">--V</div>
                                    <div class="sensor-status" id="voltageStatus">Normal</div>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- System Monitoring -->
                    <div class="sensor-section">
                        <h4>System Resources</h4>
                        <div class="sensor-cards">
                            <div class="sensor-card cpu">
                                <div class="sensor-icon">🖥️</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">CPU Usage</div>
                                    <div class="sensor-value" id="cpuValue">--%</div>
                                    <div class="sensor-status" id="cpuStatus">Normal</div>
                                </div>
                                <div class="progress-bar">
                                    <div class="progress-fill" id="cpuProgress"></div>
                                </div>
                            </div>
                            <div class="sensor-card memory">
                                <div class="sensor-icon">💾</div>
                                <div class="sensor-info">
                                    <div class="sensor-label">Memory</div>
                                    <div class="sensor-value" id="memoryValue">--%</div>
                                    <div class="sensor-status" id="memoryStatus">Normal</div>
                                </div>
                                <div class="progress-bar">
                                    <div class="progress-fill" id="memoryProgress"></div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Charts Section -->
                <div class="charts-section">
                    <div class="chart-tabs">
                        <button class="chart-tab active" data-chart="temperature">Temperature</button>
                        <button class="chart-tab" data-chart="power">Power</button>
                        <button class="chart-tab" data-chart="system">System</button>
                        <button class="chart-tab" data-chart="all">All Sensors</button>
                    </div>
                    
                    <div class="chart-container">
                        <canvas id="sensorChart" width="800" height="400"></canvas>
                    </div>
                    
                    <div class="chart-legend" id="chartLegend"></div>
                </div>

                <!-- Alerts Section -->
                <div class="alerts-section">
                    <h4>System Alerts</h4>
                    <div class="alerts-container" id="alertsContainer">
                        <div class="alert-item info">
                            <span class="alert-icon">ℹ️</span>
                            <span class="alert-message">System initialized successfully</span>
                            <span class="alert-time">${new Date().toLocaleTimeString()}</span>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    initializeCharts() {
        const canvas = document.getElementById('sensorChart');
        const ctx = canvas.getContext('2d');
        
        this.chartCanvas = canvas;
        this.chartContext = ctx;
        
        // Initialize chart drawing
        this.updateChart();
    }

    startDataSimulation() {
        // Initialize with some default values
        const now = Date.now();
        for (let i = 0; i < 10; i++) {
            this.sensorData.timestamps.push(now - (10 - i) * 1000);
            this.sensorData.temperature.push(22 + Math.random() * 6);
            this.sensorData.humidity.push(45 + Math.random() * 20);
            this.sensorData.battery.push(85 + Math.random() * 10);
            this.sensorData.current.push(0.5 + Math.random() * 1.5);
            this.sensorData.voltage.push(11.8 + Math.random() * 0.8);
            this.sensorData.cpu.push(20 + Math.random() * 30);
            this.sensorData.memory.push(30 + Math.random() * 25);
        }

        // Start real-time updates
        this.updateInterval = setInterval(() => {
            if (this.isActive) {
                this.generateSensorData();
                this.updateSensorDisplay();
                this.updateChart();
            }
        }, this.updateFrequency);
    }

    generateSensorData() {
        const now = Date.now();
        
        // Add new data points
        this.sensorData.timestamps.push(now);
        
        // Environmental sensors with realistic variations
        const lastTemp = this.sensorData.temperature[this.sensorData.temperature.length - 1] || 25;
        const newTemp = lastTemp + (Math.random() - 0.5) * 2;
        this.sensorData.temperature.push(Math.max(15, Math.min(35, newTemp)));
        
        const lastHumidity = this.sensorData.humidity[this.sensorData.humidity.length - 1] || 50;
        const newHumidity = lastHumidity + (Math.random() - 0.5) * 5;
        this.sensorData.humidity.push(Math.max(20, Math.min(80, newHumidity)));
        
        // Power system sensors
        const lastBattery = this.sensorData.battery[this.sensorData.battery.length - 1] || 90;
        const newBattery = lastBattery - Math.random() * 0.05; // Gradual discharge
        this.sensorData.battery.push(Math.max(0, Math.min(100, newBattery)));
        
        this.sensorData.current.push(0.3 + Math.random() * 1.8);
        this.sensorData.voltage.push(11.5 + Math.random() * 1.0);
        
        // System resources
        const lastCpu = this.sensorData.cpu[this.sensorData.cpu.length - 1] || 30;
        const newCpu = lastCpu + (Math.random() - 0.5) * 10;
        this.sensorData.cpu.push(Math.max(5, Math.min(95, newCpu)));
        
        const lastMemory = this.sensorData.memory[this.sensorData.memory.length - 1] || 40;
        const newMemory = lastMemory + (Math.random() - 0.5) * 8;
        this.sensorData.memory.push(Math.max(10, Math.min(90, newMemory)));
        
        // Limit data points
        Object.keys(this.sensorData).forEach(key => {
            if (this.sensorData[key].length > this.maxDataPoints) {
                this.sensorData[key].shift();
            }
        });
        
        // Check for alerts
        this.checkAlerts();
    }

    updateSensorDisplay() {
        const latest = this.getLatestValues();
        
        // Update temperature
        document.getElementById('tempValue').textContent = `${latest.temperature.toFixed(1)}°C`;
        document.getElementById('tempStatus').textContent = this.getTemperatureStatus(latest.temperature);
        
        // Update humidity
        document.getElementById('humidityValue').textContent = `${latest.humidity.toFixed(1)}%`;
        document.getElementById('humidityStatus').textContent = this.getHumidityStatus(latest.humidity);
        
        // Update battery
        document.getElementById('batteryValue').textContent = `${latest.battery.toFixed(1)}%`;
        document.getElementById('batteryStatus').textContent = this.getBatteryStatus(latest.battery);
        this.updateBatteryIndicator(latest.battery);
        
        // Update current
        document.getElementById('currentValue').textContent = `${latest.current.toFixed(2)}A`;
        document.getElementById('currentStatus').textContent = this.getCurrentStatus(latest.current);
        
        // Update voltage
        document.getElementById('voltageValue').textContent = `${latest.voltage.toFixed(1)}V`;
        document.getElementById('voltageStatus').textContent = this.getVoltageStatus(latest.voltage);
        
        // Update CPU
        document.getElementById('cpuValue').textContent = `${latest.cpu.toFixed(1)}%`;
        document.getElementById('cpuStatus').textContent = this.getCpuStatus(latest.cpu);
        this.updateProgressBar('cpuProgress', latest.cpu);
        
        // Update memory
        document.getElementById('memoryValue').textContent = `${latest.memory.toFixed(1)}%`;
        document.getElementById('memoryStatus').textContent = this.getMemoryStatus(latest.memory);
        this.updateProgressBar('memoryProgress', latest.memory);
    }

    getLatestValues() {
        return {
            temperature: this.sensorData.temperature[this.sensorData.temperature.length - 1] || 0,
            humidity: this.sensorData.humidity[this.sensorData.humidity.length - 1] || 0,
            battery: this.sensorData.battery[this.sensorData.battery.length - 1] || 0,
            current: this.sensorData.current[this.sensorData.current.length - 1] || 0,
            voltage: this.sensorData.voltage[this.sensorData.voltage.length - 1] || 0,
            cpu: this.sensorData.cpu[this.sensorData.cpu.length - 1] || 0,
            memory: this.sensorData.memory[this.sensorData.memory.length - 1] || 0
        };
    }

    // Status helper methods
    getTemperatureStatus(temp) {
        if (temp < 18) return 'Cold';
        if (temp > 30) return 'Hot';
        return 'Normal';
    }

    getHumidityStatus(humidity) {
        if (humidity < 30) return 'Low';
        if (humidity > 70) return 'High';
        return 'Normal';
    }

    getBatteryStatus(battery) {
        if (battery < 20) return 'Critical';
        if (battery < 50) return 'Low';
        return 'Good';
    }

    getCurrentStatus(current) {
        if (current > 2.0) return 'High';
        return 'Normal';
    }

    getVoltageStatus(voltage) {
        if (voltage < 11.0) return 'Low';
        if (voltage > 12.8) return 'High';
        return 'Normal';
    }

    getCpuStatus(cpu) {
        if (cpu > 80) return 'High';
        if (cpu > 60) return 'Medium';
        return 'Normal';
    }

    getMemoryStatus(memory) {
        if (memory > 85) return 'High';
        if (memory > 70) return 'Medium';
        return 'Normal';
    }

    updateBatteryIndicator(level) {
        const indicator = document.getElementById('batteryLevel');
        if (indicator) {
            indicator.style.width = `${level}%`;
            indicator.className = 'battery-level';
            if (level < 20) indicator.classList.add('critical');
            else if (level < 50) indicator.classList.add('low');
            else indicator.classList.add('good');
        }
    }

    updateProgressBar(elementId, value) {
        const progressBar = document.getElementById(elementId);
        if (progressBar) {
            progressBar.style.width = `${value}%`;
            progressBar.className = 'progress-fill';
            if (value > 80) progressBar.classList.add('high');
            else if (value > 60) progressBar.classList.add('medium');
            else progressBar.classList.add('normal');
        }
    }

    updateChart() {
        const ctx = this.chartContext;
        const canvas = this.chartCanvas;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Chart dimensions
        const padding = 60;
        const chartWidth = canvas.width - 2 * padding;
        const chartHeight = canvas.height - 2 * padding;
        
        // Draw chart based on current tab
        switch (this.currentChart) {
            case 'temperature':
                this.drawLineChart(ctx, padding, chartWidth, chartHeight, 
                    [{ data: this.sensorData.temperature, color: '#ff6b6b', label: 'Temperature (°C)' }]);
                break;
            case 'power':
                this.drawLineChart(ctx, padding, chartWidth, chartHeight, [
                    { data: this.sensorData.battery, color: '#4ecdc4', label: 'Battery (%)' },
                    { data: this.sensorData.current.map(v => v * 50), color: '#45b7d1', label: 'Current (A×50)' },
                    { data: this.sensorData.voltage.map(v => v * 8), color: '#f9ca24', label: 'Voltage (V×8)' }
                ]);
                break;
            case 'system':
                this.drawLineChart(ctx, padding, chartWidth, chartHeight, [
                    { data: this.sensorData.cpu, color: '#6c5ce7', label: 'CPU (%)' },
                    { data: this.sensorData.memory, color: '#a29bfe', label: 'Memory (%)' }
                ]);
                break;
            case 'all':
                this.drawLineChart(ctx, padding, chartWidth, chartHeight, [
                    { data: this.sensorData.temperature.map(v => v * 3), color: '#ff6b6b', label: 'Temp (°C×3)' },
                    { data: this.sensorData.battery, color: '#4ecdc4', label: 'Battery (%)' },
                    { data: this.sensorData.cpu, color: '#6c5ce7', label: 'CPU (%)' }
                ]);
                break;
        }
        
        this.updateChartLegend();
    }

    drawLineChart(ctx, padding, width, height, datasets) {
        // Draw grid
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        
        // Vertical grid lines
        for (let i = 0; i <= 10; i++) {
            const x = padding + (width * i / 10);
            ctx.beginPath();
            ctx.moveTo(x, padding);
            ctx.lineTo(x, padding + height);
            ctx.stroke();
        }
        
        // Horizontal grid lines
        for (let i = 0; i <= 5; i++) {
            const y = padding + (height * i / 5);
            ctx.beginPath();
            ctx.moveTo(padding, y);
            ctx.lineTo(padding + width, y);
            ctx.stroke();
        }
        
        // Draw axes
        ctx.strokeStyle = '#666';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(padding, padding);
        ctx.lineTo(padding, padding + height);
        ctx.lineTo(padding + width, padding + height);
        ctx.stroke();
        
        // Draw data lines
        datasets.forEach(dataset => {
            if (dataset.data.length < 2) return;
            
            ctx.strokeStyle = dataset.color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            
            const maxValue = Math.max(...dataset.data, 100);
            const minValue = Math.min(...dataset.data, 0);
            const range = maxValue - minValue || 1;
            
            dataset.data.forEach((value, index) => {
                const x = padding + (width * index / (dataset.data.length - 1));
                const y = padding + height - ((value - minValue) / range * height);
                
                if (index === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            });
            
            ctx.stroke();
            
            // Draw data points
            ctx.fillStyle = dataset.color;
            dataset.data.forEach((value, index) => {
                const x = padding + (width * index / (dataset.data.length - 1));
                const y = padding + height - ((value - minValue) / range * height);
                
                ctx.beginPath();
                ctx.arc(x, y, 3, 0, 2 * Math.PI);
                ctx.fill();
            });
        });
        
        // Draw labels
        ctx.fillStyle = '#fff';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        
        // Y-axis labels
        for (let i = 0; i <= 5; i++) {
            const y = padding + (height * i / 5);
            const value = (100 - (i * 20)).toString();
            ctx.fillText(value, padding - 20, y + 4);
        }
        
        // X-axis labels (time)
        const now = Date.now();
        for (let i = 0; i <= 5; i++) {
            const x = padding + (width * i / 5);
            const timeAgo = Math.floor((5 - i) * 10);
            ctx.fillText(`-${timeAgo}s`, x, padding + height + 20);
        }
    }

    updateChartLegend() {
        const legend = document.getElementById('chartLegend');
        const datasets = this.getDatasetsByChart(this.currentChart);
        
        legend.innerHTML = datasets.map(dataset => 
            `<span class="legend-item">
                <span class="legend-color" style="background-color: ${dataset.color}"></span>
                <span class="legend-label">${dataset.label}</span>
            </span>`
        ).join('');
    }

    getDatasetsByChart(chartType) {
        switch (chartType) {
            case 'temperature':
                return [{ color: '#ff6b6b', label: 'Temperature (°C)' }];
            case 'power':
                return [
                    { color: '#4ecdc4', label: 'Battery (%)' },
                    { color: '#45b7d1', label: 'Current (A×50)' },
                    { color: '#f9ca24', label: 'Voltage (V×8)' }
                ];
            case 'system':
                return [
                    { color: '#6c5ce7', label: 'CPU (%)' },
                    { color: '#a29bfe', label: 'Memory (%)' }
                ];
            case 'all':
                return [
                    { color: '#ff6b6b', label: 'Temp (°C×3)' },
                    { color: '#4ecdc4', label: 'Battery (%)' },
                    { color: '#6c5ce7', label: 'CPU (%)' }
                ];
            default:
                return [];
        }
    }

    checkAlerts() {
        const latest = this.getLatestValues();
        
        if (latest.battery < 20) {
            this.addAlert('warning', 'Low battery level detected', 'Battery');
        }
        
        if (latest.temperature > 32) {
            this.addAlert('warning', 'High temperature detected', 'Temperature');
        }
        
        if (latest.cpu > 85) {
            this.addAlert('error', 'High CPU usage detected', 'System');
        }
        
        if (latest.memory > 90) {
            this.addAlert('error', 'High memory usage detected', 'System');
        }
    }

    addAlert(type, message, source) {
        const alertId = `${type}-${Date.now()}`;
        const alert = {
            id: alertId,
            type: type,
            message: message,
            source: source,
            timestamp: new Date()
        };
        
        // Check if we already have this alert recently
        const recentAlert = this.alertHistory.find(a => 
            a.message === message && 
            (Date.now() - a.timestamp.getTime()) < 30000 // 30 seconds
        );
        
        if (!recentAlert) {
            this.alertHistory.unshift(alert);
            if (this.alertHistory.length > this.maxAlerts) {
                this.alertHistory.pop();
            }
            this.updateAlertsDisplay();
        }
    }

    updateAlertsDisplay() {
        const container = document.getElementById('alertsContainer');
        
        container.innerHTML = this.alertHistory.map(alert => {
            const icon = alert.type === 'error' ? '❌' : 
                        alert.type === 'warning' ? '⚠️' : 'ℹ️';
            
            return `
                <div class="alert-item ${alert.type}">
                    <span class="alert-icon">${icon}</span>
                    <span class="alert-message">${alert.message}</span>
                    <span class="alert-time">${alert.timestamp.toLocaleTimeString()}</span>
                </div>
            `;
        }).join('');
    }

    bindEvents() {
        // Control buttons
        document.getElementById('pauseBtn')?.addEventListener('click', () => this.togglePause());
        document.getElementById('resetBtn')?.addEventListener('click', () => this.resetData());
        document.getElementById('exportBtn')?.addEventListener('click', () => this.exportData());
        
        // Chart tabs
        document.querySelectorAll('.chart-tab').forEach(tab => {
            tab.addEventListener('click', (e) => this.switchChart(e.target.dataset.chart));
        });
    }

    togglePause() {
        this.isActive = !this.isActive;
        const btn = document.getElementById('pauseBtn');
        const icon = btn.querySelector('.btn-icon');
        const text = btn.querySelector('.btn-text');
        
        if (this.isActive) {
            icon.textContent = '⏸';
            text.textContent = 'Pause';
        } else {
            icon.textContent = '▶️';
            text.textContent = 'Resume';
        }
    }

    resetData() {
        // Clear all sensor data
        Object.keys(this.sensorData).forEach(key => {
            this.sensorData[key] = [];
        });
        
        // Clear alerts
        this.alertHistory = [];
        this.updateAlertsDisplay();
        
        // Add initialization alert
        this.addAlert('info', 'System data reset successfully', 'System');
        
        // Restart data generation
        this.startDataSimulation();
    }

    exportData() {
        const data = {
            timestamp: new Date().toISOString(),
            sensorData: this.sensorData,
            alerts: this.alertHistory
        };
        
        const blob = new Blob([JSON.stringify(data, null, 2)], {
            type: 'application/json'
        });
        
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `sensor-data-${Date.now()}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        this.addAlert('info', 'Sensor data exported successfully', 'System');
    }

    switchChart(chartType) {
        this.currentChart = chartType;
        
        // Update tab appearance
        document.querySelectorAll('.chart-tab').forEach(tab => {
            tab.classList.remove('active');
        });
        document.querySelector(`[data-chart="${chartType}"]`).classList.add('active');
        
        // Update chart
        this.updateChart();
    }

    destroy() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
        
        this.isActive = false;
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = SensorDashboard;
}