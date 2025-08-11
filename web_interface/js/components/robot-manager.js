class RobotManager {
    constructor(rosConnector, mapViewer) {
        this.rosConnector = rosConnector;
        this.mapViewer = mapViewer;
        this.robots = new Map();
        this.robotCards = new Map();
        this.controlPads = new Map();
        this.maxRobots = 4;
        this.nextPort = 9090;
        this.colorPalette = ["#51cf66", "#339af0", "#ffd43b", "#ff6b6b"];
        this.usedColors = new Set();
    }

    async loadDefaultRobots() {
        try {
            const response = await fetch('./config/default-robots.json');
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const config = await response.json();
            
            this.maxRobots = config.settings?.max_robots || 4;
            this.nextPort = config.settings?.starting_port || 9090;
            this.colorPalette = config.settings?.color_palette || this.colorPalette;
            
            // Start with no robots (empty system)
            this.updateConnectionStatusIndicator();
            this.log('Configuration loaded successfully - ready to add robots', 'info');
            
        } catch (error) {
            this.log(`Config file not accessible (${error.message}) - using defaults`, 'warning');
            
            // Set defaults manually
            this.maxRobots = 4;
            this.nextPort = 9090;
            this.colorPalette = ["#51cf66", "#339af0", "#ffd43b", "#ff6b6b"];
            this.usedColors = new Set();
            
            this.updateConnectionStatusIndicator();
            this.log('System initialized with default settings - ready to add robots', 'info');
        }
    }

    async addRobot(robotConfig) {
        if (this.robots.size >= this.maxRobots) {
            this.log(`Cannot add robot: Maximum of ${this.maxRobots} robots allowed`, 'warning');
            return false;
        }

        if (this.robots.has(robotConfig.id)) {
            this.log(`Robot with ID ${robotConfig.id} already exists`, 'warning');
            return false;
        }

        this.log(`Adding robot ${robotConfig.name} with config:`, 'info');
        console.log('Robot config:', robotConfig);

        // Store robot configuration
        this.robots.set(robotConfig.id, robotConfig);

        // Create robot card
        const robotCard = new RobotCard(robotConfig);
        this.robotCards.set(robotConfig.id, robotCard);
        this.log(`Robot card created for ${robotConfig.name}`, 'info');

        // Create control pad
        const controlPad = new ControlPad(
            robotConfig.id,
            robotConfig.name,
            robotConfig.color,
            this.rosConnector
        );
        this.controlPads.set(robotConfig.id, controlPad);
        this.log(`Control pad created for ${robotConfig.name}`, 'info');

        // Add to UI
        this.renderRobotCard(robotCard);
        this.renderControlPad(controlPad);

        // Update connection status indicators - IMPORTANT!
        this.updateConnectionStatusIndicator();

        this.log(`Robot ${robotConfig.name} added successfully`, 'info');
        return true;
    }

    renderRobotCard(robotCard) {
        const controlSection = document.querySelector('.control-section');
        if (!controlSection) {
            this.log('Control section not found - cannot render robot card', 'error');
            return;
        }

        // Find the ride controls element to insert before it
        const rideControls = controlSection.querySelector('.ride-controls');
        
        if (rideControls) {
            // Insert robot card before ride controls
            controlSection.insertBefore(robotCard.render(), rideControls);
            this.log(`Robot card rendered for ${robotCard.robot.name}`, 'info');
        } else {
            // If no ride controls, append at the end
            controlSection.appendChild(robotCard.render());
            this.log(`Robot card appended for ${robotCard.robot.name}`, 'info');
        }
    }

    renderControlPad(controlPad) {
        if (this.mapViewer) {
            controlPad.render(); // Make sure the control pad element is created
            this.mapViewer.addControlPad(controlPad);
            this.log(`Control pad rendered for ${controlPad.robotName}`, 'info');
        } else {
            this.log('Map viewer not found - cannot render control pad', 'error');
        }
    }

    async removeRobot(robotId) {
        if (!this.robots.has(robotId)) {
            this.log(`Robot ${robotId} not found`, 'warning');
            return false;
        }

        // Disconnect if connected
        if (this.rosConnector.isConnected(robotId)) {
            await this.disconnectRobot(robotId);
        }

        // Remove from UI
        const robotCard = this.robotCards.get(robotId);
        const controlPad = this.controlPads.get(robotId);

        if (robotCard) {
            robotCard.destroy();
            this.robotCards.delete(robotId);
        }

        if (controlPad) {
            controlPad.destroy();
            this.controlPads.delete(robotId);
        }

        // Remove from map viewer
        if (this.mapViewer) {
            this.mapViewer.removeControlPad(robotId);
        }

        // Remove from data
        this.robots.delete(robotId);

        // Update connection status indicators
        this.updateConnectionStatusIndicator();

        this.log(`Robot ${robotId} removed successfully`, 'info');
        return true;
    }

    async toggleConnection(robotId) {
        const robotConfig = this.robots.get(robotId);
        if (!robotConfig) {
            this.log(`Robot ${robotId} not found`, 'error');
            return;
        }

        if (this.rosConnector.isConnected(robotId)) {
            await this.disconnectRobot(robotId);
        } else {
            await this.connectRobot(robotId);
        }
    }

    async connectRobot(robotId) {
        const robotConfig = this.robots.get(robotId);
        const robotCard = this.robotCards.get(robotId);
        const controlPad = this.controlPads.get(robotId);

        if (!robotConfig) {
            this.log(`Robot ${robotId} not found`, 'error');
            return;
        }

        try {
            // Update UI to show connecting state
            this.log(`Connecting to ${robotConfig.name}...`, 'info');
            if (robotCard) {
                robotCard.updateStatus('Starting ROSBridge...');
                robotCard.setState('busy');
            }

            // First, try to launch ROSBridge automatically
            await this.launchROSBridge(robotConfig);

            // Wait a moment for ROSBridge to start
            await this.sleep(2000);

            // Update status
            if (robotCard) {
                robotCard.updateStatus('Connecting to robot...');
            }

            // Attempt to connect to ROS
            await this.rosConnector.connectRobot(robotConfig);
            
            // Update UI for successful connection
            if (robotCard) {
                robotCard.updateConnectionStatus(true);
                robotCard.updateStatus('Connected');
                robotCard.setState('available');
            }
            if (controlPad) {
                controlPad.setEnabled(true);
            }

            this.updateConnectionStatusIndicator();
            this.log(`${robotConfig.name} connected successfully`, 'info');
            
        } catch (error) {
            this.log(`Failed to connect to ${robotConfig.name}: ${error.message}`, 'error');
            
            // Update UI to show failed connection
            if (robotCard) {
                robotCard.updateConnectionStatus(false);
                robotCard.updateStatus('Connection failed');
                robotCard.setState('available');
            }
            if (controlPad) {
                controlPad.setEnabled(false);
            }
        }
    }

    async launchROSBridge(robotConfig) {
        this.log(`Launching ROSBridge for ${robotConfig.name}...`, 'info');
        
        // Create the launch command
        const commands = [
            `export ROS_DOMAIN_ID=${robotConfig.domain}`,
            `export ROS_DISCOVERY_SERVER="${robotConfig.discovery_server}"`,
            `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${robotConfig.rosbridge_port}`
        ];

        try {
            // Use the integrated launch server
            const response = await fetch('/launch_rosbridge', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    domain: robotConfig.domain,
                    discovery_server: robotConfig.discovery_server,
                    port: robotConfig.rosbridge_port,
                    robot_name: robotConfig.name
                })
            });

            if (response.ok) {
                const result = await response.json();
                this.log(`✅ ${result.message}`, 'info');
            } else {
                throw new Error('Launch server not available');
            }
        } catch (error) {
            // Fallback: Show user the command to run manually
            this.log(`❌ Auto-launch failed: ${error.message}`, 'error');
            this.log(`Please run this command manually in a new terminal:`, 'warning');
            this.log(`${commands.join(' && ')}`, 'info');
            throw error;
        }
    }

    showLaunchInstructions(robotConfig, commands) {
        // Create a popup with copy-pasteable commands
        const instructionsHTML = `
            <div class="modal-overlay" id="launch-instructions-modal">
                <div class="modal-content">
                    <h3>🚀 Launch ROSBridge for ${robotConfig.name}</h3>
                    <p>Please run this command in a new terminal:</p>
                    <div class="command-box">
                        <code id="launch-command">${commands.join(' && ')}</code>
                        <button class="btn btn-small" onclick="navigator.clipboard.writeText(document.getElementById('launch-command').textContent); this.textContent='Copied!';">
                            📋 Copy
                        </button>
                    </div>
                    <div class="launch-actions">
                        <button class="btn btn-secondary" onclick="document.getElementById('launch-instructions-modal').remove()">
                            I'll run it manually
                        </button>
                        <button class="btn" onclick="window.robotManager.tryAutoLaunch('${robotConfig.id}')">
                            🔄 Try Auto-Launch
                        </button>
                    </div>
                </div>
            </div>
        `;

        // Add styles for command box
        const commandStyles = `
            <style id="command-styles">
                .command-box {
                    background: #2d3748;
                    color: #e2e8f0;
                    padding: 1rem;
                    border-radius: 8px;
                    margin: 1rem 0;
                    font-family: 'Courier New', monospace;
                    position: relative;
                }
                .command-box code {
                    display: block;
                    white-space: pre-wrap;
                    word-break: break-all;
                }
                .command-box button {
                    position: absolute;
                    top: 0.5rem;
                    right: 0.5rem;
                    padding: 0.25rem 0.5rem;
                    font-size: 0.8rem;
                }
                .launch-actions {
                    display: flex;
                    gap: 1rem;
                    margin-top: 1rem;
                }
            </style>
        `;

        document.head.insertAdjacentHTML('beforeend', commandStyles);
        document.body.insertAdjacentHTML('beforeend', instructionsHTML);
    }

    async tryAutoLaunch(robotId) {
        // Close instructions modal
        const modal = document.getElementById('launch-instructions-modal');
        if (modal) modal.remove();

        // Try alternative auto-launch methods
        const robotConfig = this.robots.get(robotId);
        if (!robotConfig) return;

        this.log('Trying alternative launch methods...', 'info');

        try {
            // Method 2: Try to use system() call via a simple HTTP endpoint
            const terminalCommand = `gnome-terminal -- bash -c "export ROS_DOMAIN_ID=${robotConfig.domain}; export ROS_DISCOVERY_SERVER='${robotConfig.discovery_server}'; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${robotConfig.rosbridge_port}; exec bash"`;
            
            // This won't work from browser, but we can provide the user with options
            this.log('For auto-launch, you can:', 'info');
            this.log('1. Run the Python launcher script (if available)', 'info');
            this.log('2. Use the terminal command provided', 'info');
            this.log('3. Or manually start ROSBridge as shown', 'info');
            
        } catch (error) {
            this.log('Auto-launch not available. Please run commands manually.', 'warning');
        }
    }

    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    async disconnectRobot(robotId) {
        const robotConfig = this.robots.get(robotId);
        const robotCard = this.robotCards.get(robotId);
        const controlPad = this.controlPads.get(robotId);

        if (!robotConfig) {
            this.log(`Robot ${robotId} not found`, 'error');
            return;
        }

        try {
            this.log(`Disconnecting ${robotConfig.name}...`, 'info');
            
            this.rosConnector.disconnectRobot(robotId);
            
            // Update UI
            if (robotCard) {
                robotCard.updateConnectionStatus(false);
            }
            if (controlPad) {
                controlPad.setEnabled(false);
            }

            this.updateConnectionStatusIndicator();
            this.log(`${robotConfig.name} disconnected`, 'info');
            
        } catch (error) {
            this.log(`Error disconnecting ${robotConfig.name}: ${error.message}`, 'error');
        }
    }

    updateRobotPosition(robotId, position) {
        // Update robot card display
        const robotCard = this.robotCards.get(robotId);
        if (robotCard) {
            robotCard.updatePosition(position.x, position.y);
        }

        // Update map visualization
        if (this.mapViewer) {
            this.mapViewer.updateRobotPosition(robotId, position);
        }
    }

    updateConnectionStatusIndicator() {
        // Update connection status dots in header
        this.robots.forEach((robotConfig, robotId) => {
            const statusElement = document.getElementById(`${robotId.replace('_', '-')}-connection`);
            if (statusElement) {
                if (this.rosConnector.isConnected(robotId)) {
                    statusElement.classList.add('connected');
                } else {
                    statusElement.classList.remove('connected');
                }
            }
        });
    }

    showAddRobotDialog() {
        if (this.robots.size >= this.maxRobots) {
            this.log(`Maximum of ${this.maxRobots} robots allowed`, 'warning');
            return;
        }

        // Close any existing modal first
        this.closeAddRobotDialog();

        // Get next available color
        const availableColor = this.getNextAvailableColor();
        
        // Create modal for adding new robot
        const modalHTML = `
            <div class="modal-overlay" id="add-robot-modal">
                <div class="modal-content">
                    <h3>Add New Robot</h3>
                    <div class="form-group">
                        <label>Robot Name:</label>
                        <input type="text" id="robot-name" placeholder="e.g., Explorer, Navigator, Scout" required>
                    </div>
                    <div class="form-group">
                        <label>Domain ID:</label>
                        <input type="number" id="robot-domain" placeholder="e.g., 6, 15, 20" min="0" max="232" required>
                    </div>
                    <div class="form-group">
                        <label>Robot IP Address:</label>
                        <input type="text" id="robot-ip" placeholder="e.g., 192.168.8.206" required>
                    </div>
                    <div class="form-info">
                        <p><strong>Auto-assigned:</strong></p>
                        <p>🎨 Color: <span style="color: ${availableColor}">●</span> ${availableColor}</p>
                        <p>🔌 ROSBridge Port: ${this.nextPort}</p>
                        <p>🔗 Discovery Server: Auto-generated from Domain ID and IP</p>
                    </div>
                    <div class="form-actions">
                        <button type="button" class="btn btn-secondary" id="cancel-robot-btn">Cancel</button>
                        <button type="button" class="btn" id="submit-robot-btn">Add Robot</button>
                    </div>
                </div>
            </div>
        `;

        // Add modal styles (remove existing first)
        const existingStyles = document.getElementById('modal-styles');
        if (existingStyles) {
            existingStyles.remove();
        }

        const modalStyles = `
            <style id="modal-styles">
                .modal-overlay {
                    position: fixed;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    background: rgba(0,0,0,0.7);
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    z-index: 1000;
                }
                .modal-content {
                    background: rgba(255,255,255,0.95);
                    color: #333;
                    padding: 2rem;
                    border-radius: 12px;
                    width: 90%;
                    max-width: 500px;
                    backdrop-filter: blur(10px);
                }
                .form-group {
                    margin-bottom: 1rem;
                }
                .form-group label {
                    display: block;
                    margin-bottom: 0.5rem;
                    font-weight: 600;
                }
                .form-group input {
                    width: 100%;
                    padding: 0.5rem;
                    border: 1px solid #ccc;
                    border-radius: 4px;
                }
                .form-info {
                    background: rgba(0,0,0,0.1);
                    padding: 1rem;
                    border-radius: 8px;
                    margin: 1rem 0;
                }
                .form-info p {
                    margin: 0.25rem 0;
                    font-size: 0.9rem;
                }
                .form-actions {
                    display: flex;
                    gap: 1rem;
                    margin-top: 1.5rem;
                }
                .btn-secondary {
                    background: #6c757d;
                }
            </style>
        `;

        // Add to page
        document.head.insertAdjacentHTML('beforeend', modalStyles);
        document.body.insertAdjacentHTML('beforeend', modalHTML);

        // Bind events directly to avoid conflicts
        const submitBtn = document.getElementById('submit-robot-btn');
        const cancelBtn = document.getElementById('cancel-robot-btn');

        if (submitBtn) {
            submitBtn.addEventListener('click', () => {
                this.log('Submit button clicked', 'info');
                this.processAddRobotForm();
            });
        }

        if (cancelBtn) {
            cancelBtn.addEventListener('click', () => {
                this.log('Cancel button clicked', 'info');
                this.closeAddRobotDialog();
            });
        }

        this.log('Add robot dialog created and bound', 'info');
    }

    processAddRobotForm() {
        this.log('Processing add robot form...', 'info');
        
        const nameInput = document.getElementById('robot-name');
        const domainInput = document.getElementById('robot-domain');
        const ipInput = document.getElementById('robot-ip');
        
        this.log(`Name input: ${nameInput ? 'found' : 'NOT FOUND'}`, 'info');
        this.log(`Domain input: ${domainInput ? 'found' : 'NOT FOUND'}`, 'info');
        this.log(`IP input: ${ipInput ? 'found' : 'NOT FOUND'}`, 'info');
        
        if (!nameInput || !domainInput || !ipInput) {
            this.log('Form inputs not found', 'error');
            return;
        }
        
        const name = nameInput.value.trim();
        const domain = parseInt(domainInput.value);
        const ip = ipInput.value.trim();

        this.log(`Values - Name: '${name}', Domain: ${domain}, IP: '${ip}'`, 'info');

        // Validate required fields
        if (!name) {
            this.log('Please enter a robot name', 'error');
            return;
        }
        
        if (!domain || domain < 0 || domain > 232) {
            this.log('Please enter a valid domain ID (0-232)', 'error');
            return;
        }
        
        if (!ip) {
            this.log('Please enter a robot IP address', 'error');
            return;
        }

        // Generate robot ID from name
        const robotId = name.toLowerCase().replace(/[^a-z0-9]/g, '_');

        // Check for duplicate ID
        if (this.robots.has(robotId)) {
            this.log('Robot with similar name already exists', 'error');
            return;
        }

        // Get auto-assigned values
        const color = this.getNextAvailableColor();
        const port = this.nextPort;

        // Generate discovery server string (semicolons equal to domain ID)
        const padding = ';'.repeat(domain);
        const discoveryServer = `${padding}${ip}:11811;`;

        const robotConfig = {
            id: robotId,
            name: name,
            domain: domain,
            ip: ip,
            discovery_server: discoveryServer,
            rosbridge_port: port,
            color: color,
            enabled: false // Start disconnected
        };

        this.log(`Creating robot: ${name} (Domain: ${domain}, IP: ${ip}, Port: ${port})`, 'info');

        // Add the robot
        this.addRobotFromConfig(robotConfig);
    }

    async addRobotFromConfig(robotConfig) {
        try {
            const success = await this.addRobot(robotConfig);
            if (success) {
                this.usedColors.add(robotConfig.color);
                this.nextPort++;
                this.closeAddRobotDialog();
                this.log(`Robot "${robotConfig.name}" added successfully. Click Connect to start using it.`, 'info');
            }
        } catch (error) {
            this.log(`Failed to add robot: ${error.message}`, 'error');
        }
    }

    getNextAvailableColor() {
        for (const color of this.colorPalette) {
            if (!this.usedColors.has(color)) {
                return color;
            }
        }
        // If all colors used, cycle through them
        return this.colorPalette[this.robots.size % this.colorPalette.length];
    }

    async handleAddRobotSubmit(event) {
        if (event) {
            event.preventDefault();
        }
        
        this.log('Processing add robot request...', 'info');
        
        const nameInput = document.getElementById('robot-name');
        const domainInput = document.getElementById('robot-domain');
        const ipInput = document.getElementById('robot-ip');
        
        if (!nameInput || !domainInput || !ipInput) {
            this.log('Form inputs not found', 'error');
            return;
        }
        
        const name = nameInput.value.trim();
        const domain = parseInt(domainInput.value);
        const ip = ipInput.value.trim();

        // Validate required fields
        if (!name) {
            this.log('Please enter a robot name', 'error');
            return;
        }
        
        if (!domain || domain < 0 || domain > 232) {
            this.log('Please enter a valid domain ID (0-232)', 'error');
            return;
        }
        
        if (!ip) {
            this.log('Please enter a robot IP address', 'error');
            return;
        }

        // Generate robot ID from name
        const robotId = name.toLowerCase().replace(/[^a-z0-9]/g, '_');

        // Check for duplicate ID
        if (this.robots.has(robotId)) {
            this.log('Robot with similar name already exists', 'error');
            return;
        }

        // Get auto-assigned values
        const color = this.getNextAvailableColor();
        const port = this.nextPort;

        // Generate discovery server string (semicolons equal to domain ID)
        const padding = ';'.repeat(domain);
        const discoveryServer = `${padding}${ip}:11811;`;

        const robotConfig = {
            id: robotId,
            name: name,
            domain: domain,
            ip: ip,
            discovery_server: discoveryServer,
            rosbridge_port: port,
            color: color,
            enabled: false // Start disconnected
        };

        this.log(`Creating robot: ${name} (Domain: ${domain}, IP: ${ip}, Port: ${port})`, 'info');

        try {
            const success = await this.addRobot(robotConfig);
            if (success) {
                this.usedColors.add(color);
                this.nextPort++;
                this.closeAddRobotDialog();
                this.log(`Robot "${name}" added successfully. Click Connect to start using it.`, 'info');
            }
        } catch (error) {
            this.log(`Failed to add robot: ${error.message}`, 'error');
        }
    }

    closeAddRobotDialog() {
        const modal = document.getElementById('add-robot-modal');
        if (modal) {
            modal.remove();
        }
    }

    getRobotCount() {
        return this.robots.size;
    }

    getConnectedRobotCount() {
        let count = 0;
        this.robots.forEach((config, robotId) => {
            if (this.rosConnector.isConnected(robotId)) {
                count++;
            }
        });
        return count;
    }

    getConnectedRobots() {
        const connected = [];
        this.robots.forEach((config, robotId) => {
            if (this.rosConnector.isConnected(robotId)) {
                connected.push({ id: robotId, config });
            }
        });
        return connected;
    }

    async disconnectAllRobots() {
        const disconnectPromises = [];
        this.robots.forEach((config, robotId) => {
            if (this.rosConnector.isConnected(robotId)) {
                disconnectPromises.push(this.disconnectRobot(robotId));
            }
        });
        
        await Promise.all(disconnectPromises);
        this.log('All robots disconnected', 'info');
    }

    async connectAllRobots() {
        const connectPromises = [];
        this.robots.forEach((config, robotId) => {
            if (!this.rosConnector.isConnected(robotId)) {
                connectPromises.push(this.connectRobot(robotId));
            }
        });
        
        await Promise.allSettled(connectPromises);
        this.log('Connection attempts completed for all robots', 'info');
    }

    log(message, level = 'info') {
        if (window.logger) {
            window.logger.log(message, level);
        } else {
            console.log(`[${level.toUpperCase()}] ${message}`);
        }
    }
}

// Export for global use
window.RobotManager = RobotManager;
