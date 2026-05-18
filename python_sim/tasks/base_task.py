class BaseTask:
    """Base class for all teleoperation tasks"""
   
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.completed = False
   
    def setup(self):
        """Called once after model is loaded — set up task-specific state"""
        raise NotImplementedError
   
    def step(self, ee_body_id):
        """Called every simulation step — update task state"""
        raise NotImplementedError
   
    def get_contact_geoms(self):
        """Return list of geom IDs to check for force feedback"""
        raise NotImplementedError
   
    def is_complete(self):
        """Return True if task is finished"""
        return self.completed
   
    def get_status(self):
        """Return a string describing current task status"""
        raise NotImplementedError