# Snapcraft coding style guidelines

When writing code for snapcraft, we try to follow a set of rules that will lead
to consistency and readability.

This is a permanent work in progress, and sometimes being too strict with the
rules can end up making things actually less readable. So when you disagree
with one of the rules, please talk to us and help us making it better.

Some of the rules are enforced with static tests. You can read the [TESTING][1]
document for more information and details about how to run the static suite of
tests. Some other rules are only socially enforced during code reviews.

## black

This code base adheres to black[2].
You can `snap install black --edge` to install the linter and formatter onto your host.

## Multiline strings

For multiline strings, we prefer to use `textwrap.dedent`:

    # end first line with \ to avoid the empty line!
    s = textwrap.dedent("""\
        hello
          world
        """)
    print(repr(s))  # prints 'hello\n  world\n'

(from https://docs.python.org/3/library/textwrap.html#textwrap.dedent)

## Errors

Error messages must say what happened, why it happened and what you can do to
fix it.

## Tests

* When asserting for equality, we prefer to use the `Equals` matcher from
  testtools:

    ```
    self.assertThat(actual, Equals(expected))
    ```

* When writing unit tests that raise errors, the tests should only check the
  class of the exception raised and it's attributes, not the format of the
  error message. The formatting of the exception as a string should be
  tested only once, in the module tests/unit/test_errors.py

[1]: TESTING.md
[2]: https://github.com/ambv/black
