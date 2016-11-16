[![Build Status][travis-image]][travis-url] [![Coverage Status][coveralls-image]][coveralls-url]

# Snapcraft

Snapcraft is a delightful packaging tool

Snapcraft helps you assemble a whole project in a single tree out of
many pieces. It can drive a very wide range of build and packaging systems,
so that you can simply list all the upstream projects you want and have
them built and installed together as a single tree.

![Snapcraft Overview][overview-image]

For example, say you want to make a product that includes PyPI packages,
Node.js packages from NPM, Java, and a bunch of daemons written in C and
C++ that are built with autotools, snapcraft would make assembling the
final tree very easy.

Snapcraft allows easy crafting of snap packages for the [snappy Ubuntu Core](http://ubuntu.com/snappy)
transactional update system.

## More Information

* [Introduction](docs/intro.md) to all the details about the concepts behind snapcraft.
* [Hacking guide](HACKING.md) to contribute if you're interested in developing Snapcraft.
* [Launchpad](https://bugs.launchpad.net/snapcraft) to submit bugs or issues.

## Get in touch

We're friendly! Talk to us on [Rocket Chat](https://rocket.ubuntu.com/channel/snapcraft)
or on [our mailing list](https://lists.snapcraft.io/mailman/listinfo/snapcraft).

Get news and stay up to date on [Twitter](https://twitter.com/snapcraftio),
[Google+](https://plus.google.com/+SnapcraftIo) or
[Facebook](https://www.facebook.com/snapcraftio).

[travis-image]: https://travis-ci.org/snapcore/snapcraft.svg?branch=master
[travis-url]: https://travis-ci.org/snapcore/snapcraft

[coveralls-image]: https://coveralls.io/repos/snapcore/snapcraft/badge.svg?branch=master&service=github
[coveralls-url]: https://coveralls.io/github/snapcore/snapcraft?branch=master

[overview-image]: https://rawgit.com/snapcore/snapcraft/master/docs/images/snapcraft_overview.svg
