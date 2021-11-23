from typing import Any, Dict, List, Sequence, Union

Grammar = Sequence[Union[str, Dict[str, Any]]]
CallStack = List["Statement"]

from ._statement import Statement  # noqa: F401
