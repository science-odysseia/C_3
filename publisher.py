class Publisher(Node):
    def __init__(self):
        self.create_publisher(Image, 'image_topic', 10)
        self.create_publisher(String, 'data_topic', 10)
        self.timer = self.create_timer(1.0, self.loop)

    def loop(self):
        self.publish_data()
        self.publish_image()