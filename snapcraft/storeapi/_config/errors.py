from ..errors import StoreError


class InvalidLoginConfig(StoreError):

    fmt = "Invalid login config: {error}"

    def __init__(self, error):
        super().__init__(error=error)
