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
}
