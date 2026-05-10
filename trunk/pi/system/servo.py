class Servo:
    def __init__(self, id):

        self.id = id                        #char
        self.active = False                 #bool, init inactive and off
        self.direction = False              

    def set_command(self, active, direction):
        self.active = active

        if not active:
            self.direction = False
            return

        self.direction = direction
