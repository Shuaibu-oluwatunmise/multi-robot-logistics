// Main application class
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
            this.log('Multi-Robot Logistics System starting...', 'info');

            // Initialize core components
            this.initLogger();
            this.initRosConnector();
            this.initMapViewer();
            this.initRobotManager();
            this.initRideControls();

            // Load robots and connect
            await this.robotManager.loadDefaultRobots();

            // Set up global references
            this.setupGlobalReferences();

            // Bind global event handlers
            this.bindGlobalEvents();

            this.initialized = true;
            this.log('Application initialized successfully', 'info');

        } catch (error) {
            this.log('Failed to initialize application: ' + error.message, 'error');
            console.error('Initialization error:', error);
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

    initRideControls() {
        // Bind ride request functionality
        const requestBtn = document.getElementById('request-ride-btn');
        if (requestBtn) {
            requestBtn.addEventListener('click', () => this.handleRideRequest());
        }

        // Update initial button state
        this.updateRideControlsState();
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

        // Keyboard shortcuts
        document.addEventListener('keydown', (event) => {
            if (event.ctrlKey) {
                switch (event.key) {
                    case 'n':
                        event.preventDefault();
                        if (this.robotManager && this.robotManager.getRobotCount() < 4) {
                            this.robotManager.showAddRobotDialog();
                        }
                        break;
                    case 'r':
                        event.preventDefault();
                        this.handleRideRequest();
                        break;
                    case 'c':
                        event.preventDefault();
                        if (this.mapViewer) {
                            this.mapViewer.clearLocations();
                        }
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

    handleRideRequest() {
        const pickupLocation = this.mapViewer.getPickupLocation();
        const destinationLocation = this.mapViewer.getDestinationLocation();

        if (!pickupLocation || !destinationLocation) {
            this.log('Please set both pickup and destination locations', 'warning');
            return;
        }

        // Get connected robots
        const connectedRobots = this.robotManager.getConnectedRobots();
        
        if (connectedRobots.length === 0) {
            this.log('No robots available - please connect at least one robot', 'warning');
            return;
        }

        // Calculate distances and find closest robot
        let closestRobot = null;
        let shortestDistance = Infinity;

        connectedRobots.forEach(({ id, config }) => {
            const robotPosition = this.mapViewer.robotPositions.get(id);
            if (robotPosition) {
                const distance = this.calculateDistance(
                    pickupLocation.x, pickupLocation.y,
                    robotPosition.x, robotPosition.y
                );
                
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    closestRobot = { id, config, distance };
                }
            }
        });

        if (closestRobot) {
            this.log(`Ride requested from (${pickupLocation.x}, ${pickupLocation.y}) to (${destinationLocation.x}, ${destinationLocation.y})`, 'info');
            this.log(`${closestRobot.config.name} assigned (distance: ${closestRobot.distance.toFixed(2)}m)`, 'info');
            
            // Update robot status
            const robotCard = this.robotManager.robotCards.get(closestRobot.id);
            if (robotCard) {
                robotCard.setState('assigned');
                robotCard.updateStatus('En route to pickup');
            }

            // Clear locations for next ride
            setTimeout(() => {
                this.mapViewer.clearLocations();
                
                // Reset robot status after "ride"
                if (robotCard) {
                    robotCard.setState('available');
                    robotCard.updateStatus('Idle');
                }
            }, 5000); // Simulate 5-second ride

        } else {
            this.log('No robots with position data available', 'error');
        }
    }

    calculateDistance(x1, y1, x2, y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    updateRideControlsState() {
        // This will be called periodically to update UI state
        if (this.mapViewer) {
            this.mapViewer.updateRideControls();
        }
    }

    // System control methods
    async startMapping() {
        const connectedRobots = this.robotManager.getConnectedRobots();
        
        if (connectedRobots.length === 0) {
            this.log('No robots connected - cannot start mapping', 'warning');
            return;
        }

        this.log(`Starting mapping for ${connectedRobots.length} robot(s)`, 'info');
        
        // TODO: Implement actual SLAM launching
        // This would integrate with the launch system
        connectedRobots.forEach(({ id, config }) => {
            const robotCard = this.robotManager.robotCards.get(id);
            if (robotCard) {
                robotCard.updateStatus('Mapping');
            }
        });
    }

    async stopMapping() {
        this.log('Stopping mapping for all robots', 'info');
        
        // TODO: Implement SLAM stopping
        this.robotManager.robots.forEach((config, robotId) => {
            const robotCard = this.robotManager.robotCards.get(robotId);
            if (robotCard) {
                robotCard.updateStatus('Idle');
            }
        });
    }

    // Utility methods
    getSystemStatus() {
        return {
            robotCount: this.robotManager.getRobotCount(),
            connectedRobots: this.robotManager.getConnectedRobotCount(),
            hasPickupLocation: !!this.mapViewer.getPickupLocation(),
            hasDestinationLocation: !!this.mapViewer.getDestinationLocation(),
            initialized: this.initialized
        };
    }

    log(message, level = 'info') {
        if (this.logger) {
            this.logger.log(message, level);
        }
    }
}

// Initialize application when page loads
document.addEventListener('DOMContentLoaded', async () => {
    // Create and initialize the main application
    const app = new MultiRobotApp();
    await app.init();
    
    // Make app globally available for debugging
    window.app = app;
});

// Export for module use if needed
if (typeof module !== 'undefined' && module.exports) {
    module.exports = MultiRobotApp;
}
