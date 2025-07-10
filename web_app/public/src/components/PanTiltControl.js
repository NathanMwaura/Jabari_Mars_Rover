class PanTiltControl {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.currentPan = 0;
        this.currentTilt = 0;
        this.isDragging = false;
        this.autoTracking = false;
        this.presets = [
            { name: 'Home', pan: 0, tilt: 0 },
            { name: 'Left', pan: -90, tilt: 0 },
            { name: 'Right', pan: 90, tilt: 0 },
            { name: 'Up', pan: 0, tilt: 45 },
            { name: 'Down', pan: 0, tilt: -45 }
        ];
        
        this.init();
    }

    init() {
        this.createPanTiltInterface();
        this.bindEvents();
        this.updateDisplay();
    }

    createPanTiltInterface() {
        this.container.innerHTML = `
            <div class="pantilt-container">
                <div class="pantilt-header">
                    <h3>Pan/Tilt Control</h3>
                    <div class="pantilt-status">
                        <span class="status-indicator connected">
                            <span class="status-dot"></span>
                            <span class="status-text">Connected</span>
                        </span>
                    </div>
                </div>
                
                <div class="pantilt-main">
                    <div class="joystick-container">
                        <div class="joystick-background" id="joystickBg">
                            <div class="joystick-handle" id="joystickHandle"></div>
                            <div class="joystick-crosshair-h"></div>
                            <div class="joystick-crosshair-v"></div>
                        </div>
                        <div class="joystick-label">Virtual Joystick</div>
                    </div>
                    
                    <div class="pantilt-info">
                        <div class="position-display">
                            <div class="position-row">
                                <label>Pan:</label>
                                <span id="panValue" class="position-value">0°</span>
                                <div class="position-bar">
                                    <div class="position-fill" id="panFill" style="left: 50%"></div>
                                </div>
                            </div>
                            <div class="position-row">
                                <label>Tilt:</label>
                                <span id="tiltValue" class="position-value">0°</span>
                                <div class="position-bar">
                                    <div class="position-fill" id="tiltFill" style="left: 50%"></div>
                                </div>
                            </div>
                        </div>
                        
                        <div class="speed-controls">
                            <div class="speed-control">
                                <label for="panSpeed">Pan Speed:</label>
                                <input type="range" id="panSpeed" min="1" max="100" value="50" class="speed-slider">
                                <span id="panSpeedValue">50%</span>
                            </div>
                            <div class="speed-control">
                                <label for="tiltSpeed">Tilt Speed:</label>
                                <input type="range" id="tiltSpeed" min="1" max="100" value="50" class="speed-slider">
                                <span id="tiltSpeedValue">50%</span>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="pantilt-controls">
                    <div class="preset-controls">
                        <h4>Preset Positions</h4>
                        <div class="preset-buttons" id="presetButtons">
                            ${this.presets.map(preset => `
                                <button class="preset-btn" data-pan="${preset.pan}" data-tilt="${preset.tilt}">
                                    ${preset.name}
                                </button>
                            `).join('')}
                        </div>
                    </div>
                    
                    <div class="pantilt-actions">
                        <button id="homeBtn" class="action-btn primary">
                            <i class="icon-home"></i> Home Position
                        </button>
                        <button id="autoTrackBtn" class="action-btn toggle">
                            <i class="icon-target"></i> Auto Track
                        </button>
                        <button id="stopBtn" class="action-btn danger">
                            <i class="icon-stop"></i> Stop
                        </button>
                    </div>
                    
                    <div class="manual-controls">
                        <h4>Manual Control</h4>
                        <div class="direction-pad">
                            <button class="dir-btn" id="upBtn" data-direction="up">
                                <i class="icon-up"></i>
                            </button>
                            <div class="dir-row">
                                <button class="dir-btn" id="leftBtn" data-direction="left">
                                    <i class="icon-left"></i>
                                </button>
                                <button class="dir-btn center" id="centerBtn">
                                    <i class="icon-center"></i>
                                </button>
                                <button class="dir-btn" id="rightBtn" data-direction="right">
                                    <i class="icon-right"></i>
                                </button>
                            </div>
                            <button class="dir-btn" id="downBtn" data-direction="down">
                                <i class="icon-down"></i>
                            </button>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    bindEvents() {
        // Joystick events
        const joystickBg = document.getElementById('joystickBg');
        const joystickHandle = document.getElementById('joystickHandle');
        
        joystickBg.addEventListener('mousedown', (e) => this.startJoystickDrag(e));
        joystickBg.addEventListener('touchstart', (e) => this.startJoystickDrag(e));
        
        document.addEventListener('mousemove', (e) => this.updateJoystickDrag(e));
        document.addEventListener('touchmove', (e) => this.updateJoystickDrag(e));
        
        document.addEventListener('mouseup', () => this.endJoystickDrag());
        document.addEventListener('touchend', () => this.endJoystickDrag());
        
        // Preset buttons
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const pan = parseInt(e.target.dataset.pan);
                const tilt = parseInt(e.target.dataset.tilt);
                this.moveToPosition(pan, tilt);
            });
        });
        
        // Action buttons
        document.getElementById('homeBtn').addEventListener('click', () => this.goHome());
        document.getElementById('autoTrackBtn').addEventListener('click', () => this.toggleAutoTrack());
        document.getElementById('stopBtn').addEventListener('click', () => this.stopMovement());
        
        // Direction pad
        document.querySelectorAll('.dir-btn').forEach(btn => {
            if (btn.dataset.direction) {
                btn.addEventListener('mousedown', () => this.startContinuousMove(btn.dataset.direction));
                btn.addEventListener('mouseup', () => this.stopContinuousMove());
                btn.addEventListener('mouseleave', () => this.stopContinuousMove());
                
                btn.addEventListener('touchstart', () => this.startContinuousMove(btn.dataset.direction));
                btn.addEventListener('touchend', () => this.stopContinuousMove());
            }
        });
        
        document.getElementById('centerBtn').addEventListener('click', () => this.goHome());
        
        // Speed sliders
        document.getElementById('panSpeed').addEventListener('input', (e) => {
            document.getElementById('panSpeedValue').textContent = e.target.value + '%';
        });
        
        document.getElementById('tiltSpeed').addEventListener('input', (e) => {
            document.getElementById('tiltSpeedValue').textContent = e.target.value + '%';
        });
    }

    startJoystickDrag(e) {
        this.isDragging = true;
        this.updateJoystickDrag(e);
    }

    updateJoystickDrag(e) {
        if (!this.isDragging) return;
        
        const joystickBg = document.getElementById('joystickBg');
        const joystickHandle = document.getElementById('joystickHandle');
        const rect = joystickBg.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        let clientX, clientY;
        if (e.touches) {
            clientX = e.touches[0].clientX;
            clientY = e.touches[0].clientY;
        } else {
            clientX = e.clientX;
            clientY = e.clientY;
        }
        
        const x = clientX - rect.left - centerX;
        const y = clientY - rect.top - centerY;
        
        // Constrain to circle
        const distance = Math.sqrt(x * x + y * y);
        const maxDistance = centerX - 15; // Account for handle size
        
        let constrainedX = x;
        let constrainedY = y;
        
        if (distance > maxDistance) {
            constrainedX = (x / distance) * maxDistance;
            constrainedY = (y / distance) * maxDistance;
        }
        
        // Update handle position
        joystickHandle.style.transform = `translate(${constrainedX}px, ${constrainedY}px)`;
        
        // Calculate pan/tilt values
        const panPercent = constrainedX / maxDistance;
        const tiltPercent = -constrainedY / maxDistance; // Invert Y axis
        
        this.currentPan = panPercent * 180; // -180 to 180 degrees
        this.currentTilt = tiltPercent * 90; // -90 to 90 degrees
        
        this.updateDisplay();
        this.sendPanTiltCommand(this.currentPan, this.currentTilt);
    }

    endJoystickDrag() {
        if (!this.isDragging) return;
        
        this.isDragging = false;
        
        // Return handle to center
        const joystickHandle = document.getElementById('joystickHandle');
        joystickHandle.style.transform = 'translate(0px, 0px)';
        
        // Return to home position
        this.currentPan = 0;
        this.currentTilt = 0;
        this.updateDisplay();
        this.sendPanTiltCommand(0, 0);
    }

    startContinuousMove(direction) {
        this.continuousDirection = direction;
        this.continuousInterval = setInterval(() => {
            this.executeContinuousMove(direction);
        }, 100);
    }

    stopContinuousMove() {
        if (this.continuousInterval) {
            clearInterval(this.continuousInterval);
            this.continuousInterval = null;
        }
        this.continuousDirection = null;
    }

    executeContinuousMove(direction) {
        const speed = 2; // degrees per step
        
        switch (direction) {
            case 'up':
                this.currentTilt = Math.min(this.currentTilt + speed, 90);
                break;
            case 'down':
                this.currentTilt = Math.max(this.currentTilt - speed, -90);
                break;
            case 'left':
                this.currentPan = Math.max(this.currentPan - speed, -180);
                break;
            case 'right':
                this.currentPan = Math.min(this.currentPan + speed, 180);
                break;
        }
        
        this.updateDisplay();
        this.sendPanTiltCommand(this.currentPan, this.currentTilt);
    }

    moveToPosition(pan, tilt) {
        this.currentPan = Math.max(-180, Math.min(180, pan));
        this.currentTilt = Math.max(-90, Math.min(90, tilt));
        
        this.updateDisplay();
        this.sendPanTiltCommand(this.currentPan, this.currentTilt);
        
        console.log(`Moving to position: Pan=${this.currentPan}°, Tilt=${this.currentTilt}°`);
    }

    goHome() {
        this.moveToPosition(0, 0);
    }

    toggleAutoTrack() {
        this.autoTracking = !this.autoTracking;
        const btn = document.getElementById('autoTrackBtn');
        
        if (this.autoTracking) {
            btn.classList.add('active');
            btn.innerHTML = '<i class="icon-target"></i> Auto Track ON';
            this.startAutoTracking();
        } else {
            btn.classList.remove('active');
            btn.innerHTML = '<i class="icon-target"></i> Auto Track';
            this.stopAutoTracking();
        }
        
        console.log(`Auto tracking: ${this.autoTracking ? 'ON' : 'OFF'}`);
    }

    startAutoTracking() {
        // Simulate auto tracking with random movements
        this.autoTrackInterval = setInterval(() => {
            const randomPan = (Math.random() - 0.5) * 60; // ±30 degrees
            const randomTilt = (Math.random() - 0.5) * 30; // ±15 degrees
            this.moveToPosition(randomPan, randomTilt);
        }, 3000);
    }

    stopAutoTracking() {
        if (this.autoTrackInterval) {
            clearInterval(this.autoTrackInterval);
            this.autoTrackInterval = null;
        }
    }

    stopMovement() {
        this.stopContinuousMove();
        this.stopAutoTracking();
        this.autoTracking = false;
        document.getElementById('autoTrackBtn').classList.remove('active');
        document.getElementById('autoTrackBtn').innerHTML = '<i class="icon-target"></i> Auto Track';
        
        console.log('All movement stopped');
    }

    updateDisplay() {
        // Update position values
        document.getElementById('panValue').textContent = this.currentPan.toFixed(1) + '°';
        document.getElementById('tiltValue').textContent = this.currentTilt.toFixed(1) + '°';
        
        // Update position bars
        const panPercent = ((this.currentPan + 180) / 360) * 100;
        const tiltPercent = ((this.currentTilt + 90) / 180) * 100;
        
        document.getElementById('panFill').style.left = panPercent + '%';
        document.getElementById('tiltFill').style.left = tiltPercent + '%';
    }

    sendPanTiltCommand(pan, tilt) {
        // In a real implementation, this would send commands to ROS2
        // For demo purposes, we'll just log the commands
        console.log(`Pan/Tilt Command: Pan=${pan.toFixed(1)}°, Tilt=${tilt.toFixed(1)}°`);
        
        // Simulate sending to ROS2 interface
        if (window.ros2Interface) {
            window.ros2Interface.sendPanTiltCommand(pan, tilt);
        }
    }

    destroy() {
        this.stopMovement();
        if (this.continuousInterval) {
            clearInterval(this.continuousInterval);
        }
        if (this.autoTrackInterval) {
            clearInterval(this.autoTrackInterval);
        }
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PanTiltControl;
}