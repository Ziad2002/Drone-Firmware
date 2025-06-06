import tornado.ioloop
import tornado.web
import tornado.websocket
import os

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")
        
class ImageWebSocket(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    async def on_message(self, message):
        if isinstance(message, bytes):
            with open("static/capture.jpg", "wb") as f:
                f.write(message)
            print("Image received via WebSocket")
        else:
            print(f"Unexpected text: {message}")


class ControlWebSocket(tornado.websocket.WebSocketHandler):
    clients = set()
    
    def check_origin(self, origin):
      return True

    async def open(self):
      self.clients.add(self)
      print("Frontend WebSocket connected")

    async def on_message(self, message):
        for client in self.clients:
          print(client)  
          client.write_message(message)

    def on_close(self):
        print("Frontend WebSocket disconnected")
        self.clients.remove(self)

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/esp32_drone", ControlWebSocket), 
        (r"/esp32_image", ImageWebSocket),
    ],
    template_path=os.path.join(os.path.dirname(__file__), "templates"),
    static_path=os.path.join(os.path.dirname(__file__), "static"))

if __name__ == "__main__":
    app = make_app()
    app.listen(8888)
    print("Server running.....")
    tornado.ioloop.IOLoop.current().start()
    
