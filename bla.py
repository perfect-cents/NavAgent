import re
import itertools
from string import ascii_lowercase

EOF, NUMBER, START, END, COMMA, NEWLINE = 'EOF', 'NUMBER', 'START', 'END', 'COMMA', 'NEWLINE'

KEYWORDS = {'start' : START, 'end' : END}

class Token(object):
    def __init__(self, type_, value):
        self.type = type_
        self.value = value

    def __str__(self):
        return 'Token({type_}, {value})'.format(type_=self.type, value=repr(self.value))


    def __repr__(self):
        return self.__str__()


class Lexer(object):
    def __init__(self, text):
        self.pos = 0
        self.text = text

        self.current_char = self.text[0]
        self.last_char = None

    def error(self, msg=None):
        raise Exception('LEXER_ERROR[', msg, ']')

    def move(self):
        self.pos += 1
        if self.pos > len(self.text) - 1:
            self.last_char = self.current_char
            self.current_char = None
        else:
            self.last_char = self.current_char
            self.current_char = self.text[self.pos]

    def peek(self):
        peek_pos = self.pos + 1
        if peek_pos > len(self.text) - 1:
            return None
        else:
            return self.text[peek_pos]

    def eof(self):
        if self.current_char is not None:
            return False
        return True

    def numberize(self):
        result = self.current_char
        num = re.compile(r'^[-+]?[0-9]*?\.[0-9]+$')

        while not self.eof() and num.match(result):
            self.move()
            print result
            print self.current_char
            result = result + self.current_char

        return float(result)

    def wordize(self):
        result = self.current_char

        while not self.eof() and self.current_char.isalpha():
            self.move()
            result += self.current_char

        return result

    def scan(self):
        while not self.eof():
            if self.current_char.isspace():
                while not (self.eof() or  self.current_char == '\n') and self.current_char.isspace:
                    self.move()

            if self.current_char.isdigit() or self.current_char == '.':
                return Token(NUMBER, self.numberize())

            if self.current_char == ',':
                return Token(COMMA, ',')

            if self.current_char == '\n':
                return Token(NEWLINE, '\n')

            if self.current_char.isalpha():
                word = self.wordize()
                return Token(KEYWORDS[word], word)

            self.error('Unlexable Input : {}'.format(self.current_char))

        return Token(EOF, None)


class Point():
    def __init__(self, lon, lat, alt, name, seq):
        self.lon = lon
        self.lat = lat
        self.alt = alt
        self.name = name
        self.seq = seq
        self.points = []
        self.connections = {}


def letter_gen():
    """ A char generator that uses ascii chars for a base 26 number sysetem."""
    size = 1
    while True:
        for char in itertools.product(ascii_lowercase, repeat=size):
            yield "".join(char)
        size += 1

POINT_NAMER = letter_gen()

class Parser(object):
    def __init__(self, lexer):
        self.lexer = lexer

        self.current_token = self.lexer.scan()
        self.point_namer = letter_gen

    def error(self, msg=None):
        raise Exception('PARSER_ERROR[', msg, ']')

    def eat(self, token_type, ignore=False):
        if self.current_token.type == token_type:
            self.current_token = self.lexer.scan()
            return True
        elif not ignore:
            self.error('Unexpected Token : {}'.format(self.current_token.type))
        return False

    def prs(self):
        while self.current_token.type in (NUMBER, COMMA, START, END):
            lon = self.current_token.value
            self.eat(NUMBER)
            self.eat(COMMA)
            lat = self.current_token.value
            self.eat(NUMBER)
            self.eat(COMMA)
            alt = self.current_token.value
            self.eat(NUMBER)

            for key in (START, END):
                seq = self.current_token.value
                if self.eat(key, True):
                    break
            else:
                seq = 'MIDDLE'

            self.eat(NEWLINE)

        return Point(lon, lat, alt, POINT_NAMER.next(), seq)

class Visitor():
    def visit(self, point):
        method_name = 'visit_'+type(point).__name__
        visitor = getattr(self, method_name, self.generic_visit)
        return visitor(point)

    def generic_visit(self, node):
        raise Exception('No visit_{} method'.format(type(node).__name__))

def main():
    with open('./point_csv.txt') as data:
        text = data.read()
        data.close()

    lexer = Lexer(text)
    parser = Parser(lexer)
    result = parser.prs()
    #result = interpreter.interperate()
    print result

if __name__ == '__main__':
    main()
