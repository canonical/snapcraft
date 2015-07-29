# Snapcraft Tutorial

Let's make a snap from scratch using Snapcraft! We'll pick something a little
interesting: a webcam server.

## Preparation

You'll want a webcam and a Snappy device. We'll assume you have those,
but if you need help setting up a Snappy install, there is help
[online](https://developer.ubuntu.com/en/snappy/start/).

(Even if you don't have either of those, you can still follow along. You just
won't be able to use the final snap package you create. But you'll get to see
how Snapcraft works, which is still super rewarding.)

You should also install Snapcraft:

    $ sudo add-apt-repository ppa:snappy-dev/snapcraft-daily
    $ sudo apt-get update
    $ sudo apt-get install snapcraft

## Approach

This example is easy because we won't be doing much of the heavy lifting
ourselves. We're going to integrate a couple pieces of code together to make
an interesting app.

Namely, we'll combine a web server with a webcam program and combine
them to serve a new frame every ten seconds.

### The Web Server

Go has a simple web server in its standard libraries. So let's just use that.

It's trivial to write a complete (but basic) web server in a few lines:

    package main
    import "net/http"
    func main() {
        panic(http.ListenAndServe(":8080", http.FileServer(http.Dir("."))))
    }

This will serve the current directory on port :8080. If there is an index.html
in the current directory, it will be served. Otherwise a directory listing will
be shown.

I've provided the above code in a simple GitHub
[repository](https://github.com/mikix/golang-static-http).

### The Webcam Program

There is a webcam program provided in the Ubuntu archives called `fswebcam`.
It has a lot of neat features. But all we'll be needing for now is its ability
to take a webcam freeze frame and drop it to a file by calling it like so:

    $ fswebcam output.jpg

## Snapcraft Recipe

OK, let's create a Snapcraft recipe that combines the above programs into a
useful snap.

Snapcraft reads a single file, `snapcraft.yaml`, which tells it how to combine
code. It contains a list of `parts`, or pieces of code, and some metadata for
the final snap it will create. But let's not worry about the metadata yet.

### Web Server Part

Let's start with the web server.

    parts:
      golang-static-http:
        plugin: go1.4-project
        source: git://github.com/mikix/golang-static-http

You've got a `parts` list with one item, named `golang-static-http`, but we
could call it anything. That part has a few options. A `plugin` option that
tells Snapcraft how to interpret the part. In this case, it's a Go project
using Go 1.4. And finally, a `source` option telling Snapcraft where to
download the code.

Go ahead and create `snapcraft.yaml` with the above contents in an empty
directory.

Now we can build and "stage" this recipe. Staging just means putting the output
of the parts in a common folder that has the same layout as the snap we'll
eventually create. It lets you look at how the snap is constructed and make
sure everything is in place.

    $ snapcraft stage

You'll see a bunch of output, including Snapcraft downloading the Go compiler.
It will use this to compile the code found on GitHub. Eventually when it is
done, you'll be able to inspect the ./stage folder and see the web server
executable sitting in ./stage/bin:

    $ ls stage/bin
    golang-static-http

### Webcam Part

Now let's add the webcam program `fswebcam` to our snap. Edit `snapcraft.yaml`
to look like:

    parts:
      golang-static-http:
        plugin: go1.4-project
        source: git://github.com/mikix/golang-static-http
      fswebcam:
        plugin: ubuntu

We've added a new part called `fswebcam` handled by the `ubuntu` plugin. That
plugin will include an Ubuntu package and its dependencies into your snap. It
will use the name of the part as the package name to include.

Now let's stage our recipe again.

    $ snapcraft stage

You'll note that Snapcraft skipped downloading and building golang-static-htpp
again because it knew it had already done so.

You'll also see Snapcraft downloading and unpacking all the Ubuntu packages
into your snap. If you look at ./stage, you'll see a lot more files now:

    $ ls stage
    bin  etc  lib  usr  var

### Gluing the Parts Together

OK, so we have the two programs in our staging area. But how do we make them
work together?

We'll write a tiny little script that runs the server and `fswebcam` together:

    #!/bin/sh
    set -e

    cd "$SNAPP_APP_DATA_PATH"

    golang-static-http &

    while :; do
        fswebcam shot.jpeg
        sleep 10
    done

Save the above as `webcam-webui` and make it executable:

    $ chmod a+x webcam-webui

Alright, let's put this script in our snap too:

    parts:
      golang-static-http:
        plugin: go1.4-project
        source: git://github.com/mikix/golang-static-http
      fswebcam:
        plugin: ubuntu
      glue:
        plugin: copy
        files:
          webcam-webui: bin/webcam-webui

The `copy` plugin takes a list of files to just directly copy without building
or downloading anything. In this case, we just want to put our glue script in
the bin/ directory.

If we run Snapcraft again, we won't be surprised:

    $ snapcraft stage

We should now see both the web server and our script in stage/bin (the webcam
program is in stage/usr/bin since it came from Ubuntu):

    $ ls stage/bin
    golang-static-http  webcam-webui

### Package Metadata

"But how do we actually make a snap?", you may be wondering. To do that, we
need some metadata files required by Snappy that describe how our code should
be installed and run.

You can read all about the [format of this metadata](https://developer.ubuntu.com/en/snappy/guides/packaging-format-apps/),
but we'll assume here that you're already familiar.

Let's make a new subdirectory called `meta` and put our Snappy package files
there. Here's the contents of `meta/package.yaml`:

    name: webcam-webui
    version: 1
    vendor: You <you@example.com>
    services:
    - name: webcam-webui
      start: bin/webcam-webui

And a very simple `meta/readme.md`:

    # Webcam web UI

    Exposes your webcam over a web UI

Now that we have that sorted, we can tell Snapcraft where to find our metadata:

    parts:
      golang-static-http:
        plugin: go1.4-project
        source: git://github.com/mikix/golang-static-http
      fswebcam:
        plugin: ubuntu
      glue:
        plugin: copy
        files:
          webcam-webui: bin/webcam-webui
    snappy-metadata: meta

And tell Snapcraft to actually make the snap package:

    $ snapcraft assemble

You should now have a `webcam-webui_1_amd64.snap` file sitting in your
directory (assuming you are running on amd64). Congratulations!
