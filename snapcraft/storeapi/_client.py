import requests
from requests.adapters import HTTPAdapter
from requests.exceptions import RetryError
from requests.packages.urllib3.util.retry import Retry
import urllib.parse
import os

from . import _agent
from . import errors


class Client():
    """A base class to define clients for the ols servers.
    This is a simple wrapper around requests.Session so we inherit all good
    bits while providing a simple point for tests to override when needed.
    """

    def __init__(self, conf, root_url):
        self.conf = conf
        self.root_url = root_url
        self.session = requests.Session()
        # Setup max retries for all store URLs and the CDN
        retries = Retry(total=int(os.environ.get('STORE_RETRIES', 5)),
                        backoff_factor=int(os.environ.get('STORE_BACKOFF', 2)),
                        status_forcelist=[104, 500, 502, 503, 504])
        self.session.mount('http://', HTTPAdapter(max_retries=retries))
        self.session.mount('https://', HTTPAdapter(max_retries=retries))

        self._snapcraft_headers = {
            'User-Agent': _agent.get_user_agent(),
        }

    def request(self, method, url, params=None, headers=None, **kwargs):
        """Overriding base class to handle the root url."""
        # Note that url may be absolute in which case 'root_url' is ignored by
        # urljoin.

        if headers:
            headers.update(self._snapcraft_headers)
        else:
            headers = self._snapcraft_headers

        final_url = urllib.parse.urljoin(self.root_url, url)
        try:
            response = self.session.request(
                method, final_url, headers=headers,
                params=params, **kwargs)
        except RetryError as e:
            raise errors.StoreRetryError(e) from e

        return response

    def get(self, url, **kwargs):
        return self.request('GET', url, **kwargs)

    def post(self, url, **kwargs):
        return self.request('POST', url, **kwargs)

    def put(self, url, **kwargs):
        return self.request('PUT', url, **kwargs)
