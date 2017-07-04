import unittest

try:
    from catkin.terminal_color import ansi, enable_ANSI_colors, fmt, sanitize
except ImportError as e:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(e)
    )

try:
    char = unichr
except NameError:
    char = chr


class TerminalColorTest(unittest.TestCase):

    def test_terminal_colors(self):
        # since other test might disable ansi colors
        # we need to ensure it is enabled
        enable_ANSI_colors()
        assert ansi('reset') != ''
        test = '@_This is underlined@|'
        rslt = '\033[4mThis is underlined\033[0m\033[0m'
        assert fmt(test) == rslt
        test = 'This has bad stuff @! @/ @_ @| OK!'
        test = sanitize(test)
        rslt = 'This has bad stuff @! @/ @_ @| OK!\033[0m'
        assert fmt(test) == rslt
        test = char(2018)
        test = sanitize(test)
        rslt = char(2018)
        assert test == rslt
