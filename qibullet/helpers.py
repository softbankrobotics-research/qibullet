import pybullet


class GravityHelper:
    """
    INTERNAL CLASS, Only the qibullet API should use it. To update the gravity,
    SimulationManager objects should be used
    """
    GRAVITY_DICT = dict()

    @classmethod
    def getGravity(cls, physics_client):
        """
        INTERNAL METHOD, static getter for the gravity. This method should only
        be used by other components of the qiBullet API (the sensors for
        instance). If the required simulated instance doesn't exist, the method
        will return None

        Parameters:
            physics_client - The id of the desired simulated instance

        Returns:
            gravity - The gravity vector as a List of 3 floats
        """
        try:
            return cls.GRAVITY_DICT[physics_client]

        except KeyError:
            return None

    @classmethod
    def updateGravity(cls, physics_client, gravity):
        """
        INTERNAL METHOD, Updates (or add) a gravity value in the gravity dict

        Parameters:
            physics_client - The id of the simulated instance in which the
            gravity is updated
            gravity - The gravity vector update (List of 3 floats)
        """
        pybullet.setGravity(
            gravity[0],
            gravity[1],
            gravity[2],
            physicsClientId=physics_client)

        cls.GRAVITY_DICT[physics_client] = gravity

    @classmethod
    def removeGravity(cls, physics_client):
        """
        INTERNAL METHOD, Removes a gravity value from the gravity dict. To be
        called when the simulated instance is killed. If the required simulated
        instance does not exist, the method won't do anything

        Parameters:
            physics_client - The id of the simulated instance being removed
        """
        try:
            del cls.GRAVITY_DICT[physics_client]

        except KeyError:
            pass
