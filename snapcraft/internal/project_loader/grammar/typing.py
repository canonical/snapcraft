from typing import Any, Dict, List, Union

Grammar = List[Union[str, Dict[str, Any]]]
CallStack = List["Statement"]

from ._statement import Statement  # noqa: F401
