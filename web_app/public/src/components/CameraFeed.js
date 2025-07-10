class CameraFeed {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.isStreaming = false;
        this.isRecording = false;
        this.currentQuality = 'medium';
        this.isFullscreen = false;
        this.demoCanvas = null;
        this.animationId = null;
        
        this.init();
    }

    init() {
        this.createCameraInterface();
        this.startDemoFeed();
        this.bindEvents();
    }

    createCameraInterface() {
        this.container.innerHTML = `
            <div class="camera-container">
                <div class="camera-header">
                    <h3>Camera Feed</h3>
                    <div class="camera-controls">
                        <select id="qualitySelect" class="control-select">
                            <option value="low">Low (480p)</option>
                            <option value="medium" selected>Medium (720p)</option>
                            <option value="high">High (1080p)</option>
                        </select>
                        <button id="recordBtn" class="control-btn record-btn">
                            <i class="icon-record"></i> Record
                        </button>
                        <button id="fullscreenBtn" class="control-btn">
                            <i class="icon-fullscreen"></i> Fullscreen
                        </button>
                    </div>
                </div>
                
                <div class="camera-viewport" id="cameraViewport">
                    <canvas id="demoCanvas" width="640" height="480"></canvas>
                    <div class="camera-overlay">
                        <div class="status-indicator ${this.isStreaming ? 'streaming' : 'offline'}">
                            <span class="status-dot"></span>
                            <span class="status-text">${this.isStreaming ? 'LIVE' : 'OFFLINE'}</span>
                        </div>
                        <div class="camera-info">
                            <div class="info-item">FPS: <span id="fpsCounter">30</span></div>
                            <div class="info-item">Quality: <span id="qualityDisplay">720p</span></div>
                            <div class="info-item">Latency: <span id="latencyDisplay">45ms</span></div>
                        </div>
                    </div>
                </div>
                
                <div class="camera-footer">
                    <div class="recording-controls" style="display: none;">
                        <span class="recording-indicator">
                            <span class="recording-dot"></span>
                            Recording: <span id="recordingTime">00:00</span>
                        </span>
                        <button id="stopRecordBtn" class="control-btn stop-btn">Stop Recording</button>
                    </div>
                </div>
            </div>
        `;
    }

    startDemoFeed() {
        this.demoCanvas = document.getElementById('demoCanvas');
        const ctx = this.demoCanvas.getContext('2d');
        let frame = 0;
        
        const drawFrame = () => {
            // Clear canvas
            ctx.clearRect(0, 0, this.demoCanvas.width, this.demoCanvas.height);
            
            // Draw simulated robot camera view
            const gradient = ctx.createLinearGradient(0, 0, this.demoCanvas.width, this.demoCanvas.height);
            gradient.addColorStop(0, '#1a1a2e');
            gradient.addColorStop(0.5, '#16213e');
            gradient.addColorStop(1, '#0f0f23');
            
            ctx.fillStyle = gradient;
            ctx.fillRect(0, 0, this.demoCanvas.width, this.demoCanvas.height);
            
            // Draw simulated environment elements
            this.drawSimulatedEnvironment(ctx, frame);
            
            // Draw crosshair
            this.drawCrosshair(ctx);
            
            // Update frame counter
            frame++;
            
            if (this.isStreaming) {
                this.animationId = requestAnimationFrame(drawFrame);
            }
        };
        
        this.isStreaming = true;
        drawFrame();
        this.updateStatus();
    }

    drawSimulatedEnvironment(ctx, frame) {
        // Draw moving objects to simulate camera feed
        const time = frame * 0.02;
        
        // Draw simulated walls/obstacles
        ctx.strokeStyle = '#4a9eff';
        ctx.lineWidth = 2;
        
        // Moving obstacle
        const obstacleX = 100 + Math.sin(time) * 50;
        const obstacleY = 200 + Math.cos(time * 0.5) * 30;
        
        ctx.strokeRect(obstacleX, obstacleY, 40, 60);
        
        // Draw simulated floor grid
        ctx.strokeStyle = '#333355';
        ctx.lineWidth = 1;
        
        for (let i = 0; i < this.demoCanvas.width; i += 40) {
            ctx.beginPath();
            ctx.moveTo(i, 0);
            ctx.lineTo(i, this.demoCanvas.height);
            ctx.stroke();
        }
        
        for (let i = 0; i < this.demoCanvas.height; i += 40) {
            ctx.beginPath();
            ctx.moveTo(0, i);
            ctx.lineTo(this.demoCanvas.width, i);
            ctx.stroke();
        }
        
        // Draw simulated sensor data
        ctx.fillStyle = '#ff6b6b';
        ctx.font = '12px monospace';
        ctx.fillText(`Distance: ${(50 + Math.sin(time) * 20).toFixed(1)}cm`, 10, 30);
        ctx.fillText(`Battery: ${(85 + Math.sin(time * 0.1) * 5).toFixed(1)}%`, 10, 50);
    }

    drawCrosshair(ctx) {
        const centerX = this.demoCanvas.width / 2;
        const centerY = this.demoCanvas.height / 2;
        
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 1;
        
        // Horizontal line
        ctx.beginPath();
        ctx.moveTo(centerX - 20, centerY);
        ctx.lineTo(centerX + 20, centerY);
        ctx.stroke();
        
        // Vertical line
        ctx.beginPath();
        ctx.moveTo(centerX, centerY - 20);
        ctx.lineTo(centerX, centerY + 20);
        ctx.stroke();
        
        // Center dot
        ctx.fillStyle = '#00ff00';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 2, 0, 2 * Math.PI);
        ctx.fill();
    }

    bindEvents() {
        // Quality selector
        document.getElementById('qualitySelect').addEventListener('change', (e) => {
            this.changeQuality(e.target.value);
        });

        // Record button
        document.getElementById('recordBtn').addEventListener('click', () => {
            this.toggleRecording();
        });

        // Fullscreen button
        document.getElementById('fullscreenBtn').addEventListener('click', () => {
            this.toggleFullscreen();
        });

        // Stop recording button
        document.getElementById('stopRecordBtn').addEventListener('click', () => {
            this.stopRecording();
        });
    }

    changeQuality(quality) {
        this.currentQuality = quality;
        const qualityMap = {
            'low': { width: 640, height: 480, display: '480p' },
            'medium': { width: 1280, height: 720, display: '720p' },
            'high': { width: 1920, height: 1080, display: '1080p' }
        };
        
        const settings = qualityMap[quality];
        this.demoCanvas.width = settings.width;
        this.demoCanvas.height = settings.height;
        
        document.getElementById('qualityDisplay').textContent = settings.display;
        
        console.log(`Camera quality changed to: ${settings.display}`);
    }

    toggleRecording() {
        if (!this.isRecording) {
            this.startRecording();
        } else {
            this.stopRecording();
        }
    }

    startRecording() {
        this.isRecording = true;
        this.recordingStartTime = Date.now();
        
        document.querySelector('.recording-controls').style.display = 'flex';
        document.getElementById('recordBtn').classList.add('recording');
        document.getElementById('recordBtn').innerHTML = '<i class="icon-stop"></i> Recording...';
        
        this.updateRecordingTime();
        console.log('Recording started');
    }

    stopRecording() {
        this.isRecording = false;
        
        document.querySelector('.recording-controls').style.display = 'none';
        document.getElementById('recordBtn').classList.remove('recording');
        document.getElementById('recordBtn').innerHTML = '<i class="icon-record"></i> Record';
        
        console.log('Recording stopped');
    }

    updateRecordingTime() {
        if (!this.isRecording) return;
        
        const elapsed = Date.now() - this.recordingStartTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        
        document.getElementById('recordingTime').textContent = 
            `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
        
        setTimeout(() => this.updateRecordingTime(), 1000);
    }

    toggleFullscreen() {
        const viewport = document.getElementById('cameraViewport');
        
        if (!this.isFullscreen) {
            if (viewport.requestFullscreen) {
                viewport.requestFullscreen();
            } else if (viewport.webkitRequestFullscreen) {
                viewport.webkitRequestFullscreen();
            } else if (viewport.mozRequestFullScreen) {
                viewport.mozRequestFullScreen();
            }
            this.isFullscreen = true;
        } else {
            if (document.exitFullscreen) {
                document.exitFullscreen();
            } else if (document.webkitExitFullscreen) {
                document.webkitExitFullscreen();
            } else if (document.mozCancelFullScreen) {
                document.mozCancelFullScreen();
            }
            this.isFullscreen = false;
        }
    }

    updateStatus() {
        const statusIndicator = document.querySelector('.status-indicator');
        const statusText = document.querySelector('.status-text');
        
        if (this.isStreaming) {
            statusIndicator.classList.remove('offline');
            statusIndicator.classList.add('streaming');
            statusText.textContent = 'LIVE';
        } else {
            statusIndicator.classList.remove('streaming');
            statusIndicator.classList.add('offline');
            statusText.textContent = 'OFFLINE';
        }
        
        // Simulate FPS and latency updates
        setInterval(() => {
            document.getElementById('fpsCounter').textContent = Math.floor(28 + Math.random() * 4);
            document.getElementById('latencyDisplay').textContent = Math.floor(40 + Math.random() * 20) + 'ms';
        }, 2000);
    }

    destroy() {
        this.isStreaming = false;
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = CameraFeed;
}