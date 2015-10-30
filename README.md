[![Build Status][travis-image]][travis-url] [![Coverage Status][coveralls-image]][coveralls-url]

# Snapcraft

Snapcraft is a delightful packaging tool

Snapcraft helps you assemble a whole project in a single tree out of
many pieces. It can drive a very wide range of build and packaging systems,
so that you can simply list all the upstream projects you want and have
them built and installed together as a single tree.

For example, say you want to make a product that includes PyPI packages,
Node.js packages from NPM, Java, and a bunch of daemons written in C and
C++ that are built with autotools, snapcraft would make assembling the
final tree very easy.

Snapcraft allows easy crafting of snap packages for the Ubuntu Core
transactional system.

See the [intro](https://github.com/ubuntu-core/snapcraft/blob/master/docs/intro.md)
for details about the concepts behind
snapcraft. There is also a [tutorial](https://github.com/ubuntu-core/snapcraft/blob/master/docs/intro.md). If you're interested in developing Snapcraft you should look at the [hacking guide](https://github.com/ubuntu-core/snapcraft/blob/master/HACKING.md).

[travis-image]: https://travis-ci.org/ubuntu-core/snapcraft.svg?branch=master
[travis-url]: https://travis-ci.org/ubuntu-core/snapcraft.svg?branch=master

[coveralls-image]: https://coveralls.io/repos/ubuntu-core/snapcraft/badge.svg?branch=master&service=github
[coveralls-url]: https://coveralls.io/github/ubuntu-core/snapcraft?branch=master
