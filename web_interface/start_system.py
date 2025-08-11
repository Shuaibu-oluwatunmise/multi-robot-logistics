#!/usr/bin/env python3

import subprocess
import threading
import time
import json
import os
import signal
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler, SimpleHTTPRequestHandler
from urllib.parse import urlparse
import webbrowser

class MultiRobotHandler(BaseHTTPRequestHandler):
    # Store running ROSBridge processes
    rosbridge_processes = {}
    
    def do_GET(self):
        # Serve static files (HTML, CSS, JS)
        if self.path == '/':
            self.path = '/index.html'
        
        # Remove leading slash and serve files
        file_path = self.path.lstrip('/')
        if os.path.exists(file_path):
            if file_path.endswith('.html'):
                content_type = 'text/html'
            elif file_path.endswith('.css'):
                content_type = 'text/css'
            elif file_path.endswith('.js'):
                content_type = 'application/javascript'
            elif file_path.endswith('.json'):
                content_type = 'application/json'
            else:
                content_type = 'text/plain'
                
            self.send_response(200)
            self.send_header('Content-type', content_type)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            with open(file_path, 'rb') as file:
                self.wfile.write(file.read())
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_POST(self):
        if self.path == '/launch_rosbridge':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                
                domain = data.get('domain')
                discovery_server = data.get('discovery_server')
                port = data.get('port')
                robot_name = data.get('robot_name', 'Robot')
                
                print(f"\n🚀 Auto-launching ROSBridge for {robot_name}")
                print(f"   Domain: {domain}")
                print(f"   Port: {port}")
                print(f"   Discovery: {discovery_server}")
                
                # Kill existing process for this port
                if port in self.rosbridge_processes:
                    try:
                        print(f"   Stopping existing ROSBridge on port {port}")
                        self.rosbridge_processes[port].terminate()
                        time.sleep(1)
                        if self.rosbridge_processes[port].poll() is None:
                            self.rosbridge_processes[port].kill()
                    except:
                        pass
                
                # Launch ROSBridge in new terminal
                launch_cmd = [
                    'gnome-terminal',
                    '--title', f'ROSBridge - {robot_name} (Port {port})',
                    '--',
                    'bash', '-c', 
                    f'export ROS_DOMAIN_ID={domain}; '
                    f'export ROS_DISCOVERY_SERVER="{discovery_server}"; '
                    f'echo "🤖 Starting ROSBridge for {robot_name}..."; '
                    f'echo "Domain: {domain}, Port: {port}"; '
                    f'echo ""; '
                    f'ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={port}; '
                    f'echo ""; '
                    f'echo "ROSBridge stopped. Press Enter to close..."; '
                    f'read'
                ]
                
                process = subprocess.Popen(launch_cmd)
                self.rosbridge_processes[port] = process
                
                print(f"   ✅ ROSBridge terminal opened for {robot_name}")
                
                # Send success response
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                response = {
                    'status': 'success', 
                    'message': f'ROSBridge launched for {robot_name}',
                    'port': port
                }
                self.wfile.write(json.dumps(response).encode())
                
            except Exception as e:
                print(f"❌ Error launching ROSBridge: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                response = {'status': 'error', 'message': str(e)}
                self.wfile.write(json.dumps(response).encode())
                
        elif self.path == '/stop_rosbridge':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                
                port = data.get('port')
                
                if port in self.rosbridge_processes:
                    self.rosbridge_processes[port].terminate()
                    del self.rosbridge_processes[port]
                    print(f"🛑 Stopped ROSBridge on port {port}")
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                response = {'status': 'success', 'message': f'ROSBridge stopped on port {port}'}
                self.wfile.write(json.dumps(response).encode())
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                response = {'status': 'error', 'message': str(e)}
                self.wfile.write(json.dumps(response).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_OPTIONS(self):
        # Handle CORS preflight
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

def cleanup_processes(handler):
    """Clean up all running processes"""
    print("\n🧹 Cleaning up ROSBridge processes...")
    for port, process in handler.rosbridge_processes.items():
        try:
            process.terminate()
            print(f"   Stopped ROSBridge on port {port}")
        except:
            pass

def signal_handler(signum, frame, httpd):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Shutting down Multi-Robot System...")
    cleanup_processes(httpd.RequestHandlerClass)
    httpd.shutdown()
    sys.exit(0)

def main():
    # Change to web interface directory
    web_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(web_dir)
    
    print("🤖 Multi-Robot Logistics System")
    print("=" * 50)
    print("🌐 Starting integrated web server...")
    print("🚀 ROSBridge auto-launch ready...")
    print("")
    
    # Start HTTP server
    server_address = ('', 8080)
    httpd = HTTPServer(server_address, MultiRobotHandler)
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, httpd))
    
    print(f"✅ Server running at: http://localhost:8080")
    print(f"📁 Serving files from: {web_dir}")
    print("")
    print("🎯 Usage:")
    print("   1. Open http://localhost:8080 in your browser")
    print("   2. Click 'Add Robot' to configure robots")  
    print("   3. Click 'Connect' - ROSBridge launches automatically!")
    print("   4. Use Ctrl+C to stop everything")
    print("")
    
    # Auto-open browser
    try:
        threading.Timer(1.0, lambda: webbrowser.open('http://localhost:8080')).start()
        print("🌐 Opening browser automatically...")
    except:
        print("💡 Manually open: http://localhost:8080")
    
    print("📊 Server logs:")
    print("-" * 30)
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None, httpd)

if __name__ == '__main__':
    main()
