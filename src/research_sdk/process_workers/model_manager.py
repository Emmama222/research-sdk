from research_sdk.world.model import WorldModel
from multiprocessing.managers import BaseManager

class WorldModelManager(BaseManager): pass

WorldModelManager.register('WorldModel', WorldModel)
    