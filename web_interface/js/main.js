// Main application class - MANAGER MODE ONLY
class MultiRobotApp {
    constructor() {
        this.rosConnector = null;
        this.mapViewer = null;
        this.robotManager = null;
        this.logger = null;
        this.initialized = false;
    }

    async init() {
        if (this.initialized) return;

        try {
            this.log('Fleet Management System starting...', 'info');

            // Initialize core components
            this.initLogger();
            this.initRosConnector();
            this.initMapViewer();
            this.initRobotManager();

            // Load robots and connect
            await this.robotManager.loadDefaultRobots();

            // Set up global references
            this.setupGlobalReferences();

            // Bind global event handlers
            this.bindGlobalEvents();

            this.initialized = true;
            this.log('Fleet Management System initialized successfully', 'info');

        } catch (error) {
            this.log('Failed to initialize application: ' + error.message, 'error');
            console.error('Initialization error:', error);
        }
    }
    
    log(message, level = 'info') {
        if (this.logger && typeof this.logger.log === 'function') {
            this.logger.log(message, level);
        } else {
            console.log(`[${level.toUpperCase()}] ${message}`);
        }
    }

    initLogger() {
        this.logger = {
            log: (message, level = 'info') => {
                const timestamp = new Date().toLocaleTimeString();
                const logContainer = document.getElementById('log-container');
                
                if (logContainer) {
                    const logEntry = document.createElement('div');
                    logEntry.className = `log-entry ${level}`;
                    logEntry.textContent = `${timestamp}: ${message}`;
                    
                    logContainer.appendChild(logEntry);
                    logContainer.scrollTop = logContainer.scrollHeight;
                    
                    // Keep only last 50 entries
                    while (logContainer.children.length > 50) {
                        logContainer.removeChild(logContainer.firstChild);
                    }
                } else {
                    console.log(`[${level.toUpperCase()}] ${timestamp}: ${message}`);
                }
            }
        };

        // Make logger globally available
        window.logger = this.logger;
    }

    initRosConnector() {
        this.rosConnector = new ROSConnector();
        
        // Make globally available for components
        window.rosConnector = this.rosConnector;
    }

    initMapViewer() {
        this.mapViewer = new MapViewer();
        
        // Replace existing map section
        const mapSection = document.querySelector('.map-section');
        if (mapSection) {
            mapSection.replaceWith(this.mapViewer.render());
        }

        // Make globally available
        window.mapViewer = this.mapViewer;
    }

    initRobotManager() {
        this.robotManager = new RobotManager(this.rosConnector, this.mapViewer);
        
        // Make globally available for UI interactions
        window.robotManager = this.robotManager;
    }

    setupGlobalReferences() {
        // Expose main components globally for debugging and component communication
        window.app = this;
        window.robotManager = this.robotManager;
        window.mapViewer = this.mapViewer;
        window.rosConnector = this.rosConnector;
        window.logger = this.logger;
    }

    bindGlobalEvents() {
        // Add robot button - use event delegation since robotManager might not be ready immediately
        document.addEventListener('click', (event) => {
            if (event.target.id === 'add-robot-btn') {
                event.preventDefault();
                if (this.robotManager) {
                    this.robotManager.showAddRobotDialog();
                } else {
                    this.log('Robot manager not ready yet', 'warning');
                }
            }
        });

        // Keyboard shortcuts for manager mode
        document.addEventListener('keydown', (event) => {
            if (event.ctrlKey) {
                switch (event.key) {
                    case 'n':
                        event.preventDefault();
                        if (this.robotManager && this.robotManager.getRobotCount() < 4) {
                            this.robotManager.showAddRobotDialog();
                        }
                        break;
                    case 'm':
                        event.preventDefault();
                        this.startMapping();
                        break;
                    case 's':
                        event.preventDefault();
                        this.stopMapping();
                        break;
                }
            }
        });

        // Window resize handler
        window.addEventListener('resize', () => {
            // Update map visualization on resize
            if (this.mapViewer) {
                setTimeout(() => {
                    this.mapViewer.updateMapVisualization();
                }, 100);
            }
        });

        // Before unload - disconnect all robots
        window.addEventListener('beforeunload', () => {
            if (this.robotManager) {
                this.robotManager.disconnectAllRobots();
            }
        });
    }

    // SLAM MANAGEMENT METHODS - SEPARATE METHODS, NOT INSIDE bindGlobalEvents!
    async startMapping() {
        const connectedRobots = this.robotManager.getConnectedRobots();
        
        if (connectedRobots.length === 0) {
            this.log('No robots connected - cannot start mapping', 'warning');
            return;
        }

        this.log(`Starting SLAM mapping for ${connectedRobots.length} robot(s)`, 'info');
        
        // Launch SLAM for each connected robot
        const slamPromises = connectedRobots.map(({ id, config }) => {
            return this.launchSLAMForRobot(config);
        });

        try {
            // Start all SLAM processes
            await Promise.allSettled(slamPromises);
            
            // Subscribe to map topics to get map data
            await this.subscribeToMapTopics();
            
            this.log('SLAM mapping started successfully! Drive the robots around to build maps.', 'info');
        } catch (error) {
            this.log(`Failed to start mapping: ${error.message}`, 'error');
        }
    }

    async launchSLAMForRobot(robotConfig) {
        this.log(`Launching SLAM for ${robotConfig.name}...`, 'info');
        
        try {
            // Send SLAM launch request to our Python server
            const response = await fetch('/launch_slam', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    domain: robotConfig.domain,
                    discovery_server: robotConfig.discovery_server,
                    robot_name: robotConfig.name,
                    robot_id: robotConfig.id,
                    sync: false // Use async SLAM like your working version
                })
            });

            if (response.ok) {
                const result = await response.json();
                this.log(`✅ ${result.message}`, 'info');
                
                // Update robot status
                const robotCard = this.robotManager.robotCards.get(robotConfig.id);
                if (robotCard) {
                    robotCard.updateStatus('Mapping');
                    robotCard.setState('busy');
                }
            } else {
                throw new Error('SLAM launch server not available');
            }
        } catch (error) {
            // Fallback: Show manual command with correct format
            const slamCommand = [
                `export ROS_DOMAIN_ID=${robotConfig.domain}`,
                `export ROS_DISCOVERY_SERVER=";;;;;;${robotConfig.discovery_server};"`,
                `ros2 launch turtlebot4_navigation slam.launch.py sync:=false`
            ].join(' && ');
            
            this.log(`❌ Auto SLAM launch failed for ${robotConfig.name}`, 'error');
            this.log(`Manual command: ${slamCommand}`, 'info');
            throw error;
        }
    }

    async subscribeToMapTopics() {
        this.log('Subscribing to map and pose topics...', 'info');
        
        // Subscribe to map data and robot poses from all connected robots
        const connectedRobots = this.robotManager.getConnectedRobots();
        
        connectedRobots.forEach(({ id, config }) => {
            // Subscribe to /map topic for each robot
            const mapTopic = new ROSLIB.Topic({
                ros: this.rosConnector.connections.get(id),
                name: '/map',
                messageType: 'nav_msgs/OccupancyGrid'
            });

            mapTopic.subscribe((message) => {
                this.handleMapUpdate(id, message);
            });

            // Subscribe to robot pose for position and orientation
            const poseTopic = new ROSLIB.Topic({
                ros: this.rosConnector.connections.get(id),
                name: '/pose',  // or '/odom' or '/amcl_pose' - try different topics
                messageType: 'geometry_msgs/PoseWithCovarianceStamped'
            });

            poseTopic.subscribe((message) => {
                this.handlePoseUpdate(id, message);
            });

            // Also try odometry if pose doesn't work
            const odomTopic = new ROSLIB.Topic({
                ros: this.rosConnector.connections.get(id),
                name: '/odom',
                messageType: 'nav_msgs/Odometry'
            });

            odomTopic.subscribe((message) => {
                this.handleOdomUpdate(id, message);
            });

            this.log(`Subscribed to /map and pose topics for ${config.name}`, 'info');
        });
    }
    handlePoseUpdate(robotId, poseMessage) {
        if (this.mapViewer && poseMessage.pose && poseMessage.pose.pose) {
            this.mapViewer.updateRobotPose(robotId, poseMessage.pose.pose);
            
            const pos = poseMessage.pose.pose.position;
            this.log(`${robotId} pose: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)})`, 'info');
        }
    }

    handleOdomUpdate(robotId, odomMessage) {
        if (this.mapViewer && odomMessage.pose && odomMessage.pose.pose) {
            this.mapViewer.updateRobotPose(robotId, odomMessage.pose.pose);
            
            const pos = odomMessage.pose.pose.position;
            this.log(`${robotId} odometry: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)})`, 'info');
        }
    }

    handleMapUpdate(robotId, mapData) {
        // Update the map viewer with new map data
        if (this.mapViewer) {
            this.mapViewer.updateMapData(robotId, mapData);
        }
        
        // Log map progress
        const occupiedCells = mapData.data.filter(cell => cell > 50).length;
        const totalCells = mapData.data.length;
        const mappingProgress = ((occupiedCells / totalCells) * 100).toFixed(1);
        
        this.log(`${robotId} map update: ${mappingProgress}% coverage`, 'info');
    }

    async stopMapping() {
        this.log('Stopping SLAM mapping for all robots', 'info');
        
        const connectedRobots = this.robotManager.getConnectedRobots();
        
        // Stop SLAM processes
        const stopPromises = connectedRobots.map(({ id, config }) => {
            return this.stopSLAMForRobot(config);
        });

        try {
            await Promise.allSettled(stopPromises);
            this.log('SLAM mapping stopped for all robots', 'info');
        } catch (error) {
            this.log(`Error stopping mapping: ${error.message}`, 'error');
        }
    }

    async stopSLAMForRobot(robotConfig) {
        try {
            const response = await fetch('/stop_slam', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    robot_name: robotConfig.name,
                    robot_id: robotConfig.id
                })
            });

            if (response.ok) {
                // Update robot status
                const robotCard = this.robotManager.robotCards.get(robotConfig.id);
                if (robotCard) {
                    robotCard.updateStatus('Connected');
                    robotCard.setState('available');
                }
            }
        } catch (error) {
            this.log(`Failed to stop SLAM for ${robotConfig.name}`, 'warning');
        }
    }
}

// Export for global use
window.MultiRobotApp = MultiRobotApp;
