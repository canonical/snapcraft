import logging
import requests
from requests.adapters import HTTPAdapter
from requests.exceptions import ConnectionError, RetryError
from requests.packages.urllib3.util.retry import Retry
import urllib.parse
import os

from . import _agent
from . import errors

# Set urllib3's logger to only emit errors, not warnings. Otherwise even
# retries are printed, and they're nasty.
logging.getLogger(requests.packages.urllib3.__package__).setLevel(logging.ERROR)


class Client:
    """A base class to define clients for the ols servers.
    This is a simple wrapper around requests.Session so we inherit all good
    bits while providing a simple point for tests to override when needed.
    """

    def __init__(self, conf, root_url):
        """Initialize Client object

        :param config conf: Configuration details for the client
        :param str root_url: Root url for all requests.
        :type config: snapcraft.config.Config
        """
        self.conf = conf
        self.root_url = root_url
        self.session = requests.Session()
        # Setup max retries for all store URLs and the CDN
        retries = Retry(
            total=int(os.environ.get("STORE_RETRIES", 5)),
            backoff_factor=int(os.environ.get("STORE_BACKOFF", 2)),
            status_forcelist=[104, 500, 502, 503, 504],
        )
        self.session.mount("http://", HTTPAdapter(max_retries=retries))
        self.session.mount("https://", HTTPAdapter(max_retries=retries))

        self._snapcraft_headers = {"User-Agent": _agent.get_user_agent()}

    def request(self, method, url, params=None, headers=None, **kwargs):
        """Send a request to url relative to the root url.

        :param str method: Method used for the request.
        :param str url: Appended with the root url first.
        :param list params: Query parameters to be sent along with the request.
        :param list headers: Headers to be sent along with the request.

        :return Response of the request.
        """
        # Note that url may be absolute in which case 'root_url' is ignored by
        # urljoin.

        if headers:
            headers.update(self._snapcraft_headers)
        else:
            headers = self._snapcraft_headers

        final_url = urllib.parse.urljoin(self.root_url, url)
        try:
            response = self.session.request(
                method, final_url, headers=headers, params=params, **kwargs
            )
        except (ConnectionError, RetryError) as e:
            raise errors.StoreNetworkError(e) from e

        # Handle 5XX responses generically right here, so the callers don't
        # need to worry about it.
        if response.status_code >= 500:
            raise errors.StoreServerError(response)

        return response

    def get(self, url, **kwargs):
        """Perform a GET request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self.request("GET", url, **kwargs)

    def post(self, url, **kwargs):
        """Perform a POST request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self.request("POST", url, **kwargs)

    def put(self, url, **kwargs):
        """Perform a PUT request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self.request("PUT", url, **kwargs)
