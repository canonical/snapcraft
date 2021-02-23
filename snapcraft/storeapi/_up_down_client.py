import os

from . import constants
from ._client import Client


class UpDownClient(Client):
    """The Up/Down server provide upload/download snap capabilities."""

    def __init__(self, conf):
        super().__init__(
            conf, os.environ.get("STORE_UPLOAD_URL", constants.STORE_UPLOAD_URL),
        )

    def upload(self, monitor):
        return self.post(
            "/unscanned-upload/",
            data=monitor,
            headers={
                "Content-Type": monitor.content_type,
                "Accept": "application/json",
            },
        )
