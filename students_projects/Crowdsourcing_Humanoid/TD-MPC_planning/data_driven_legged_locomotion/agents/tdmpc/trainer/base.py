class Trainer:
    """Base trainer class for TD-MPC2."""

    def __init__(self, cfg, env, agent, buffer, logger):
        self.cfg = cfg
        self.env = env
        self.agent = agent
        self.buffer = buffer
        self.logger = logger
        # print("Architecture:", self.agent.model) # TODO(my-rice): uncomment this line to print the model architecture. 
        print("Learnable parameters: {:,}".format(self.agent.model.total_params))

    def eval(self):
        """Evaluate a TD-MPC2 agent."""
        raise NotImplementedError

    def train(self):
        """Train a TD-MPC2 agent."""
        raise NotImplementedError
