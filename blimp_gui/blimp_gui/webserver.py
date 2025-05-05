from http.server import SimpleHTTPRequestHandler
import socketserver
import os
from ament_index_python.packages import get_package_share_directory

PORT = 8000

#web_dir = os.path.join(os.path.dirname(__file__), '..', 'static')
web_dir = os.path.join(get_package_share_directory('blimp_gui'), 'static')
os.chdir(web_dir)

Handler = SimpleHTTPRequestHandler
httpd = socketserver.TCPServer(("0.0.0.0", PORT), Handler)  # Bind to all interfaces
print(f"Serving web GUI at http://0.0.0.0:{PORT}")
httpd.serve_forever()

