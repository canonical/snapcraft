import itertools
from time import sleep
from threading import Thread
from queue import Queue

from progressbar import AnimatedMarker, ProgressBar, UnknownLength

import requests

from . import constants
from . import errors


class StatusTracker:

    __messages = {
        "being_processed": "Processing...",
        "ready_to_release": "Ready to release!",
        "need_manual_review": "Will need manual review...",
        "processing_upload_delta_error": "Error while processing delta...",
        "processing_error": "Error while processing...",
    }

    __error_codes = {
        "processing_error",
        "processing_upload_delta_error",
        "need_manual_review",
    }

    def __init__(self, status_details_url):
        self.__status_details_url = status_details_url

    def track(self):
        queue = Queue()
        thread = Thread(target=self._update_status, args=(queue,))
        thread.start()

        widgets = ["Processing...", AnimatedMarker()]
        progress_indicator = ProgressBar(widgets=widgets, maxval=UnknownLength)
        progress_indicator.start()

        content = {}
        for indicator_count in itertools.count():
            progress_indicator.update(indicator_count)
            if not queue.empty():
                content = queue.get()
                if isinstance(content, Exception):
                    raise content
            if content.get("processed"):
                break
            else:
                widgets[0] = self._get_message(content)
            sleep(0.1)
        progress_indicator.finish()
        # Print at the end to avoid a left over spinner artifact
        print(self._get_message(content))

        self.__content = content

        return content

    def raise_for_code(self):
        if self.__content["code"] in self.__error_codes:
            raise errors.StoreReviewError(self.__content)

    def _get_message(self, content):
        try:
            return self.__messages.get(content["code"], content["code"])
        except KeyError:
            return self.__messages.get("being_processed")

    def _update_status(self, queue):
        for content in self._get_status():
            queue.put(content)
            if content["processed"]:
                break
            sleep(constants.SCAN_STATUS_POLL_DELAY)

    def _get_status(self):
        connection_errors_allowed = 10
        while True:
            try:
                content = requests.get(self.__status_details_url).json()
            except (requests.ConnectionError, requests.HTTPError) as e:
                if not connection_errors_allowed:
                    yield e
                content = {"processed": False, "code": "being_processed"}
                connection_errors_allowed -= 1
            yield content
