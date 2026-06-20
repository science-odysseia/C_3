class Subscriber(Node):
    def __init__(self):
        self.create_subscription(Image, 'image_topic', self.cb_image, 10)
        self.create_subscription(String, 'data_topic', self.cb_data, 10)

    def cb_image(self, msg):
        show image

    def cb_data(self, msg):
        print data
