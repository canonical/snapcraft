# The MIT License
# Copyright (c) 2014 Evan Cordell.
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""Code copied from pymacaroons until the package is available for xenial.

Only the bits needed for snapcraft have been copied.
"""
import base64
import binascii
import copy
import hashlib
import hmac
import struct


class MacaroonException(Exception):
    pass


class MacaroonSerializationException(MacaroonException):
    pass


class MacaroonDeserializationException(MacaroonException):
    pass


class HashSignaturesBinder(object):

    def __init__(self, root, key=None):
        self.root = root
        self.key = key or truncate_or_pad(b'\0')

    def bind(self, discharge):
        protected = discharge.copy()
        protected.signature = self.bind_signature(
            convert_to_bytes(discharge.signature))
        return protected

    def bind_signature(self, signature):
        return hmac_concat(
            self.key,
            binascii.unhexlify(convert_to_bytes(self.root.signature)),
            binascii.unhexlify(signature)
        )


class Caveat(object):

    def __init__(self,
                 caveat_id=None,
                 verification_key_id=None,
                 location=None):
        self.caveat_id = caveat_id
        self.verification_key_id = verification_key_id
        self.location = location


class Macaroon(object):

    PACKET_PREFIX_LENGTH = 4

    def __init__(self,
                 location=None,
                 identifier=None,
                 key=None,
                 caveats=None,
                 signature=None):
        self.caveats = caveats or []
        self.location = location or ''
        self.identifier = identifier or ''
        self.signature = signature or ''
        if key:
            self.signature = create_initial_signature(
                convert_to_bytes(key), convert_to_bytes(identifier)
            )

    def copy(self):
        return copy.deepcopy(self)

    def prepare_for_request(self, macaroon):
        protected = macaroon.copy()
        return HashSignaturesBinder(self).bind(protected)

    def serialize(self):
        combined = self._packetize(b'location', self.location)
        combined += self._packetize(b'identifier', self.identifier)

        for caveat in self.caveats:
            combined += self._packetize(b'cid',
                                        convert_to_bytes(caveat.caveat_id))

            if caveat.verification_key_id and caveat.location:
                combined += self._packetize(
                    b'vid', convert_to_bytes(caveat.verification_key_id))
                combined += self._packetize(b'cl',
                                            convert_to_bytes(caveat.location))

        combined += self._packetize(
            b'signature',
            binascii.unhexlify(convert_to_bytes(self.signature))
        )
        return base64.urlsafe_b64encode(combined).decode('ascii').rstrip('=')

    @classmethod
    def deserialize(cls, serialized):
        macaroon = Macaroon()

        decoded = base64.urlsafe_b64decode(convert_to_bytes(
            serialized + "=" * (-len(serialized) % 4)
        ))

        index = 0

        while index < len(decoded):
            packet_length = int(
                struct.unpack(
                    b"4s",
                    decoded[index:index + cls.PACKET_PREFIX_LENGTH]
                )[0],
                16
            )
            packet = decoded[
                index + cls.PACKET_PREFIX_LENGTH:index + packet_length
            ]

            key, value = _depacketize(packet)

            if key == b'location':
                macaroon.location = value
            elif key == b'identifier':
                macaroon.identifier = value
            elif key == b'cid':
                macaroon.caveats.append(Caveat(caveat_id=value))
            elif key == b'vid':
                macaroon.caveats[-1].verification_key_id = value
            elif key == b'cl':
                macaroon.caveats[-1].location = value
            elif key == b'signature':
                macaroon.signature = binascii.hexlify(value)
            else:
                raise MacaroonDeserializationException(
                    'Key {key} not valid key for this format. '
                    'Value: {value}'.format(
                        key=key, value=value
                    )
                )

            index = index + packet_length

        return macaroon

    def _packetize(self, key, data):
        # The 2 covers the space and the newline
        packet_size = self.PACKET_PREFIX_LENGTH + 2 + len(key) + len(data)
        # Ignore the first two chars, 0x
        packet_size_hex = hex(packet_size)[2:]

        if packet_size > 65535:
            raise MacaroonSerializationException(
                'Packet too long for serialization. '
                'Max length is 0xFFFF (65535). '
                'Packet length: 0x{hex_length} ({length}) '
                'Key: {key}'.format(
                    key=key,
                    hex_length=packet_size_hex,
                    length=packet_size
                )
            )

        header = packet_size_hex.zfill(4).encode('ascii')
        packet_content = key + b' ' + convert_to_bytes(data) + b'\n'
        packet = struct.pack(
            convert_to_bytes("4s%ds" % len(packet_content)),
            header,
            packet_content
        )
        return packet


def _depacketize(packet):
    key = packet.split(b' ')[0]
    value = packet[len(key) + 1:-1]
    return (key, value)


def convert_to_bytes(string_or_bytes):
    if string_or_bytes is None:
        return None
    if isinstance(string_or_bytes, str):
        return string_or_bytes.encode('ascii')
    elif isinstance(string_or_bytes, bytes):
        return string_or_bytes
    else:
        raise TypeError("Must be a string or bytes object.")


def convert_to_string(string_or_bytes):
    if string_or_bytes is None:
        return None
    if isinstance(string_or_bytes, str):
        return string_or_bytes
    elif isinstance(string_or_bytes, bytes):
        return string_or_bytes.decode('ascii')
    else:
        raise TypeError("Must be a string or bytes object.")


def truncate_or_pad(byte_string, size=None):
    if size is None:
        size = 32
    byte_array = bytearray(byte_string)
    length = len(byte_array)
    if length > size:
        return bytes(byte_array[:size])
    elif length < size:
        return bytes(byte_array + b"\0" * (size - length))
    else:
        return byte_string


def hmac_digest(key, data):
    return hmac.new(
        key,
        msg=data,
        digestmod=hashlib.sha256
    ).digest()


def hmac_hex(key, data):
    dig = hmac_digest(key, data)
    return binascii.hexlify(dig)


def hmac_concat(key, data1, data2):
    hash1 = hmac_digest(key, data1)
    hash2 = hmac_digest(key, data2)
    return hmac_hex(key, hash1 + hash2)


def generate_derived_key(key):
    return hmac_digest(b'macaroons-key-generator', key)


def create_initial_signature(key, identifier):
    derived_key = generate_derived_key(key)
    return hmac_hex(derived_key, identifier)
