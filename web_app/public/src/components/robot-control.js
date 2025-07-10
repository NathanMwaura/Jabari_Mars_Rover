class RobotControl {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.isConnected = false;
        this.currentMode = 'manual';
        this.linearVelocity = 0;
        this.angularVelocity = 0;
        this.maxLinearSpeed = 2.0; // m/s
        this.maxAngularSpeed = 3.14; // rad/s
        this.isMoving = false;
        this.emergencyStop = false;
        
        this.keyStates = {};
        this.moveInterval = null;
        
        this.init();
    }

    init() {
        this.createRobotInterface();
        this.bindEvents();
        this.updateDisplay();
        this.simulateConnection();
    }

    createRobotInterface() {
        this.container.innerHTML = `
            <div class="robot-control-container">
                <div class="robot-header">
                    <h3>Robot Control</h3>
                    <div class="connection-status">
                        <span class="status-indicator ${this.isConnected ? 'connected' : 'disconnected'}">
                            <span class="status-dot"></span>
                            <span class="status-text">${this.isConnected ? 'Connected' : 'Disconnected'}</span>
                        </span>
                    </div>
                </div>
                
                <div class="emergency-section">
                    <button id="emergencyStopBtn" class="emergency-btn ${this.emergencyStop ? 'active' : ''}">
                        <i class="icon-stop">⏹</i>
                        <span>EMERGENCY STOP</span>
                    </button>
                </div>
                
                <div class="mode-selection">
                    <h4>Control Mode</h4>
                    <div class="mode-buttons">
                        <button class="mode-btn ${this.currentMode === 'manual' ? 'active' : ''}" data-mode="manual">
                            <i class="icon-manual">🎮</i> Manual
                        </button>
                        <button class="mode-btn ${this.currentMode === 'autonomous' ? 'active' : ''}" data-mode="autonomous">
                            <i class="icon-auto">🤖</i> Autonomous
                        </button>
                        <button class="mode-btn ${this.currentMode === 'waypoint' ? 'active' : ''}" data-mode="waypoint">
                            <i class="icon-waypoint">📍</i> Waypoint
                        </button>
                    </div>
                </div>
                
                <div class="velocity-controls">
                    <h4>Velocity Settings</h4>
                    <div class="velocity-sliders">
                        <div class="slider-group">
                            <label for="maxLinearSlider">Max Linear Speed: <span id="maxLinearValue">${this.maxLinearSpeed}</span> m/s</label>
                            <input type="range" id="maxLinearSlider" min="0.1" max="3.0" step="0.1" value="${this.maxLinearSpeed}">
                        </div>
                        <div class="slider-group">
                            <label for="maxAngularSlider">Max Angular Speed: <span id="maxAngularValue">${this.maxAngularSpeed}</span> rad/s</label>
                            <input type="range" id="maxAngularSlider" min="0.1" max="6.28" step="0.1" value="${this.maxAngularSpeed}">
                        </div>
                    </div>
                </div>
                
                <div class="manual-controls ${this.currentMode === 'manual' ? 'active' : 'hidden'}">
                    <h4>Manual Control</h4>
                    <div class="control-instructions">
                        <p>Use WASD keys or arrow keys to control the robot</p>
                        <div class="key-mapping">
                            <div class="key-row">
                                <span class="key">W/↑</span><span class="action">Forward</span>
                            </div>
                            <div class="key-row">
                                <span class="key">S/↓</span><span class="action">Backward</span>
                            </div>
                            <div class="key-row">
                                <span class="key">A/←</span><span class="action">Turn Left</span>
                            </div>
                            <div class="key-row">
                                <span class="key">D/→</span><span class="action">Turn Right</span>
                            </div>
                            <div class="key-row">
                                <span class="key">Space</span><span class="action">Stop</span>
                            </div>
                        </div>
                    </div>
                    
                    <div class="virtual-joystick">
                        <div class="joystick-container">
                            <div class="joystick-base" id="joystickBase">
                                <div class="joystick-handle" id="joystickHandle"></div>
                            </div>
                        </div>
                    </div>
                    
                    <div class="direction-pad">
                        <button class="direction-btn up" data-direction="forward">↑</button>
                        <div class="middle-row">
                            <button class="direction-btn left" data-direction="left">←</button>
                            <button class="direction-btn stop" data-direction="stop">⏹</button>
                            <button class="direction-btn right" data-direction="right">→</button>
                        </div>
                        <button class="direction-btn down" data-direction="backward">↓</button>
                    </div>
                </div>
                
                <div class="autonomous-controls ${this.currentMode === 'autonomous' ? 'active' : 'hidden'}">
                    <h4>Autonomous Control</h4>
                    <div class="autonomous-options">
                        <button class="auto-btn" id="startPatrolBtn">Start Patrol</button>
                        <button class="auto-btn" id="returnHomeBtn">Return Home</button>
                        <button class="auto-btn" id="exploreBtn">Explore Mode</button>
                        <button class="auto-btn" id="followLineBtn">Follow Line</button>
                    </div>
                    <div class="autonomous-status">
                        <p>Status: <span id="autonomousStatus">Idle</span></p>
                        <p>Current Task: <span id="currentTask">None</span></p>
                    </div>
                </div>
                
                <div class="waypoint-controls ${this.currentMode === 'waypoint' ? 'active' : 'hidden'}">
                    <h4>Waypoint Navigation</h4>
                    <div class="waypoint-list">
                        <div class="waypoint-header">
                            <span>Waypoints</span>
                            <button class="add-waypoint-btn" id="addWaypointBtn">+ Add</button>
                        </div>
                        <ul id="waypointList" class="waypoint-items">
                            <!-- Waypoints will be dynamically added here -->
                        </ul>
                    </div>
                    <div class="waypoint-actions">
                        <button class="waypoint-btn" id="executeWaypointsBtn">Execute Route</button>
                        <button class="waypoint-btn" id="clearWaypointsBtn">Clear All</button>
                    </div>
                </div>
                
                <div class="velocity-display">
                    <h4>Current Velocity</h4>
                    <div class="velocity-values">
                        <div class="velocity-item">
                            <span class="velocity-label">Linear:</span>
                            <span class="velocity-value" id="currentLinear">${this.linearVelocity.toFixed(2)} m/s</span>
                        </div>
                        <div class="velocity-item">
                            <span class="velocity-label">Angular:</span>
                            <span class="velocity-value" id="currentAngular">${this.angularVelocity.toFixed(2)} rad/s</span>
                        </div>
                        <div class="velocity-item">
                            <span class="velocity-label">Status:</span>
                            <span class="velocity-status" id="movementStatus">${this.isMoving ? 'Moving' : 'Stopped'}</span>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    bindEvents() {
        // Emergency stop
        const emergencyBtn = document.getElementById('emergencyStopBtn');
        emergencyBtn.addEventListener('click', () => this.toggleEmergencyStop());

        // Mode selection
        const modeButtons = document.querySelectorAll('.mode-btn');
        modeButtons.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const mode = e.target.closest('.mode-btn').dataset.mode;
                this.setMode(mode);
            });
        });

        // Velocity sliders
        const maxLinearSlider = document.getElementById('maxLinearSlider');
        const maxAngularSlider = document.getElementById('maxAngularSlider');
        
        maxLinearSlider.addEventListener('input', (e) => {
            this.maxLinearSpeed = parseFloat(e.target.value);
            document.getElementById('maxLinearValue').textContent = this.maxLinearSpeed;
        });

        maxAngularSlider.addEventListener('input', (e) => {
            this.maxAngularSpeed = parseFloat(e.target.value);
            document.getElementById('maxAngularValue').textContent = this.maxAngularSpeed;
        });

        // Direction pad buttons
        const directionButtons = document.querySelectorAll('.direction-btn');
        directionButtons.forEach(btn => {
            btn.addEventListener('mousedown', (e) => {
                const direction = e.target.dataset.direction;
                this.startMovement(direction);
            });
            btn.addEventListener('mouseup', () => this.stopMovement());
            btn.addEventListener('mouseleave', () => this.stopMovement());
        });

        // Keyboard controls
        document.addEventListener('keydown', (e) => this.handleKeyDown(e));
        document.addEventListener('keyup', (e) => this.handleKeyUp(e));

        // Virtual joystick
        this.initVirtualJoystick();

        // Autonomous controls
        this.bindAutonomousControls();

        // Waypoint controls
        this.bindWaypointControls();

        // Prevent context menu on control elements
        this.container.addEventListener('contextmenu', (e) => e.preventDefault());
    }

    initVirtualJoystick() {
        const joystickBase = document.getElementById('joystickBase');
        const joystickHandle = document.getElementById('joystickHandle');
        
        let isDragging = false;
        let baseRect;
        const maxDistance = 50;

        const startDrag = (e) => {
            if (this.emergencyStop) return;
            isDragging = true;
            baseRect = joystickBase.getBoundingClientRect();
            joystickBase.style.cursor = 'grabbing';
        };

        const drag = (e) => {
            if (!isDragging) return;

            const clientX = e.clientX || (e.touches && e.touches[0].clientX);
            const clientY = e.clientY || (e.touches && e.touches[0].clientY);

            const centerX = baseRect.left + baseRect.width / 2;
            const centerY = baseRect.top + baseRect.height / 2;

            const deltaX = clientX - centerX;
            const deltaY = clientY - centerY;
            const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            let x, y;
            if (distance <= maxDistance) {
                x = deltaX;
                y = deltaY;
            } else {
                x = (deltaX / distance) * maxDistance;
                y = (deltaY / distance) * maxDistance;
            }

            joystickHandle.style.transform = `translate(${x}px, ${y}px)`;

            // Calculate velocities based on joystick position
            const normalizedX = x / maxDistance;
            const normalizedY = -y / maxDistance; // Invert Y axis

            this.linearVelocity = normalizedY * this.maxLinearSpeed;
            this.angularVelocity = -normalizedX * this.maxAngularSpeed;

            this.updateDisplay();
            this.sendVelocityCommand();
        };

        const endDrag = () => {
            if (!isDragging) return;
            isDragging = false;
            joystickHandle.style.transform = 'translate(0px, 0px)';
            joystickBase.style.cursor = 'grab';
            
            this.linearVelocity = 0;
            this.angularVelocity = 0;
            this.updateDisplay();
            this.sendVelocityCommand();
        };

        // Mouse events
        joystickBase.addEventListener('mousedown', startDrag);
        document.addEventListener('mousemove', drag);
        document.addEventListener('mouseup', endDrag);

        // Touch events
        joystickBase.addEventListener('touchstart', startDrag);
        document.addEventListener('touchmove', drag);
        document.addEventListener('touchend', endDrag);
    }

    bindAutonomousControls() {
        document.getElementById('startPatrolBtn').addEventListener('click', () => {
            this.startAutonomousTask('patrol');
        });
        
        document.getElementById('returnHomeBtn').addEventListener('click', () => {
            this.startAutonomousTask('return_home');
        });
        
        document.getElementById('exploreBtn').addEventListener('click', () => {
            this.startAutonomousTask('explore');
        });
        
        document.getElementById('followLineBtn').addEventListener('click', () => {
            this.startAutonomousTask('follow_line');
        });
    }

    bindWaypointControls() {
        document.getElementById('addWaypointBtn').addEventListener('click', () => {
            this.addWaypoint();
        });
        
        document.getElementById('executeWaypointsBtn').addEventListener('click', () => {
            this.executeWaypoints();
        });
        
        document.getElementById('clearWaypointsBtn').addEventListener('click', () => {
            this.clearWaypoints();
        });
    }

    handleKeyDown(e) {
        if (this.emergencyStop || this.currentMode !== 'manual') return;

        const key = e.key.toLowerCase();
        this.keyStates[key] = true;

        // Prevent default browser behavior for arrow keys and space
        if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(key)) {
            e.preventDefault();
        }

        this.updateMovementFromKeys();
    }

    handleKeyUp(e) {
        const key = e.key.toLowerCase();
        this.keyStates[key] = false;

        if (key === ' ') { // Space bar - emergency stop
            this.stopMovement();
            return;
        }

        this.updateMovementFromKeys();
    }

    updateMovementFromKeys() {
        let linear = 0;
        let angular = 0;

        // Forward/Backward
        if (this.keyStates['w'] || this.keyStates['arrowup']) {
            linear = this.maxLinearSpeed;
        } else if (this.keyStates['s'] || this.keyStates['arrowdown']) {
            linear = -this.maxLinearSpeed;
        }

        // Left/Right turning
        if (this.keyStates['a'] || this.keyStates['arrowleft']) {
            angular = this.maxAngularSpeed;
        } else if (this.keyStates['d'] || this.keyStates['arrowright']) {
            angular = -this.maxAngularSpeed;
        }

        this.linearVelocity = linear;
        this.angularVelocity = angular;
        
        this.updateDisplay();
        this.sendVelocityCommand();
    }

    startMovement(direction) {
        if (this.emergencyStop) return;

        switch (direction) {
            case 'forward':
                this.linearVelocity = this.maxLinearSpeed;
                this.angularVelocity = 0;
                break;
            case 'backward':
                this.linearVelocity = -this.maxLinearSpeed;
                this.angularVelocity = 0;
                break;
            case 'left':
                this.linearVelocity = 0;
                this.angularVelocity = this.maxAngularSpeed;
                break;
            case 'right':
                this.linearVelocity = 0;
                this.angularVelocity = -this.maxAngularSpeed;
                break;
            case 'stop':
                this.stopMovement();
                return;
        }

        this.isMoving = true;
        this.updateDisplay();
        this.sendVelocityCommand();
    }

    stopMovement() {
        this.linearVelocity = 0;
        this.angularVelocity = 0;
        this.isMoving = false;
        this.updateDisplay();
        this.sendVelocityCommand();
    }

    toggleEmergencyStop() {
        this.emergencyStop = !this.emergencyStop;
        
        if (this.emergencyStop) {
            this.stopMovement();
        }

        const emergencyBtn = document.getElementById('emergencyStopBtn');
        emergencyBtn.classList.toggle('active', this.emergencyStop);
        
        this.updateDisplay();
        console.log('Emergency stop:', this.emergencyStop ? 'ACTIVATED' : 'DEACTIVATED');
    }

    setMode(mode) {
        this.currentMode = mode;
        
        // Update mode buttons
        const modeButtons = document.querySelectorAll('.mode-btn');
        modeButtons.forEach(btn => {
            btn.classList.toggle('active', btn.dataset.mode === mode);
        });

        // Show/hide control sections
        const controlSections = document.querySelectorAll('.manual-controls, .autonomous-controls, .waypoint-controls');
        controlSections.forEach(section => {
            section.classList.remove('active');
            section.classList.add('hidden');
        });

        const activeSection = document.querySelector(`.${mode}-controls`);
        if (activeSection) {
            activeSection.classList.add('active');
            activeSection.classList.remove('hidden');
        }

        // Stop movement when switching modes
        this.stopMovement();

        console.log('Control mode changed to:', mode);
    }

    startAutonomousTask(task) {
        if (this.emergencyStop) return;

        document.getElementById('autonomousStatus').textContent = 'Running';
        document.getElementById('currentTask').textContent = task.replace('_', ' ').toUpperCase();

        // Simulate autonomous behavior (in real implementation, this would send ROS commands)
        console.log('Starting autonomous task:', task);
        
        // Demo: Show some movement simulation
        setTimeout(() => {
            document.getElementById('autonomousStatus').textContent = 'Completed';
        }, 5000);
    }

    addWaypoint() {
        const waypointList = document.getElementById('waypointList');
        const waypointCount = waypointList.children.length + 1;
        
        // In real implementation, this would use current robot position
        const x = (Math.random() * 10 - 5).toFixed(2);
        const y = (Math.random() * 10 - 5).toFixed(2);
        
        const waypointElement = document.createElement('li');
        waypointElement.className = 'waypoint-item';
        waypointElement.innerHTML = `
            <span class="waypoint-info">
                <strong>Waypoint ${waypointCount}</strong><br>
                X: ${x}m, Y: ${y}m
            </span>
            <button class="remove-waypoint-btn" onclick="this.parentElement.remove()">×</button>
        `;
        
        waypointList.appendChild(waypointElement);
        console.log(`Added waypoint ${waypointCount} at (${x}, ${y})`);
    }

    executeWaypoints() {
        const waypoints = document.querySelectorAll('.waypoint-item');
        if (waypoints.length === 0) {
            alert('No waypoints to execute');
            return;
        }

        console.log(`Executing route with ${waypoints.length} waypoints`);
        // In real implementation, this would send waypoint navigation commands to ROS
    }

    clearWaypoints() {
        const waypointList = document.getElementById('waypointList');
        waypointList.innerHTML = '';
        console.log('All waypoints cleared');
    }

    sendVelocityCommand() {
        // In real implementation, this would send velocity commands to ROS2
        // For demo purposes, just log the command
        if (this.linearVelocity !== 0 || this.angularVelocity !== 0) {
            console.log(`Velocity Command - Linear: ${this.linearVelocity.toFixed(2)} m/s, Angular: ${this.angularVelocity.toFixed(2)} rad/s`);
        }
    }

    updateDisplay() {
        // Update velocity display
        document.getElementById('currentLinear').textContent = `${this.linearVelocity.toFixed(2)} m/s`;
        document.getElementById('currentAngular').textContent = `${this.angularVelocity.toFixed(2)} rad/s`;
        document.getElementById('movementStatus').textContent = this.isMoving ? 'Moving' : 'Stopped';

        // Update connection status
        const statusIndicator = document.querySelector('.status-indicator');
        const statusText = document.querySelector('.status-text');
        statusIndicator.className = `status-indicator ${this.isConnected ? 'connected' : 'disconnected'}`;
        statusText.textContent = this.isConnected ? 'Connected' : 'Disconnected';
    }

    simulateConnection() {
        // Simulate connection after a delay (for demo purposes)
        setTimeout(() => {
            this.isConnected = true;
            this.updateDisplay();
            console.log('Robot connection established (simulated)');
        }, 2000);
    }

    // Public methods for external control
    connect() {
        this.isConnected = true;
        this.updateDisplay();
    }

    disconnect() {
        this.isConnected = false;
        this.stopMovement();
        this.updateDisplay();
    }

    getStatus() {
        return {
            connected: this.isConnected,
            mode: this.currentMode,
            velocities: {
                linear: this.linearVelocity,
                angular: this.angularVelocity
            },
            emergencyStop: this.emergencyStop,
            moving: this.isMoving
        };
    }
}