import os
import urllib.parse

from ._client import Client

from . import constants


class UpDownClient(Client):
    """The Up/Down server provide upload/download snap capabilities."""

    def __init__(self, conf):
        super().__init__(
            conf,
            os.environ.get(
                "UBUNTU_STORE_UPLOAD_ROOT_URL", constants.UBUNTU_STORE_UPLOAD_ROOT_URL
            ),
        )

    def upload(self, monitor):
        return self.post(
            urllib.parse.urljoin(self.root_url, "unscanned-upload/"),
            data=monitor,
            headers={
                "Content-Type": monitor.content_type,
                "Accept": "application/json",
            },
        )
