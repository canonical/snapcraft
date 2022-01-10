import os
from urllib.parse import urljoin

import requests

from ._requests import Requests
from . import constants


class UpDownClient(Requests):
    """The Up/Down server provide upload/download snap capabilities."""

    def __init__(self, client) -> None:
        self._client = client
        self._root_url = os.getenv("STORE_UPLOAD_URL", constants.STORE_UPLOAD_URL)

    def _request(self, method, urlpath, **kwargs) -> requests.Response:
        url = urljoin(self._root_url, urlpath)
        return self._client.request(method, url, **kwargs)

    def upload(self, monitor):
        return self.post(
            "/unscanned-upload/",
            data=monitor,
            headers={
                "Content-Type": monitor.content_type,
                "Accept": "application/json",
            },
        )
