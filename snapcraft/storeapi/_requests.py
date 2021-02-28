import abc

import requests


class Requests(abc.ABC):
    @abc.abstractmethod
    def _request(self, method: str, urlpath: str, **kwargs) -> requests.Response:
        """Request using method against url."""

    def get(self, urlpath: str, **kwargs) -> requests.Response:
        """Perform a POST request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self._request("GET", urlpath, **kwargs)

    def post(self, urlpath: str, **kwargs) -> requests.Response:
        """Perform a POST request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self._request("POST", urlpath, **kwargs)

    def put(self, urlpath: str, **kwargs) -> requests.Response:
        """Perform a PUT request with the given arguments.

        The arguments are the same as for the request function,
        namely params and headers.

        :param str url: url to send the request.
        :return Response of the request.
        """
        return self._request("PUT", urlpath, **kwargs)
