# Upload to the store

So you've been working hard on your snap, and you finally have it to the point
where you're ready to share it with the world? Great! You can use Snapcraft to
upload it directly to the store.

## The grade option

`grade` in your `snapcraft.yaml` file can be either `stable` or `devel`. It
essentially tags the resulting snap for life with the grade it should have.

How is this useful? Imagine you are developing a snap and have your continuous
integration setup to always push to the `edge` channel; in this case one would
set `grade: devel`. This ensures that this snap is not accidentally published
to the `stable` channels.

A snap with `grade: devel` cannot be released to the `stable` or `candidate`
channels.


## Build the snap

Get into the directory containing the `snapcraft.yaml` file, and do the following:

    $ snapcraft snap
    [...]
    Snapped foo_1_amd64.snap


## Authenticate to the store

First of all, you need to give Snapcraft permission to communicate with the
store on your behalf:

    $ snapcraft login
    Enter your Ubuntu One SSO credentials.
    Email: me@example.com
    Password:
    Second-factor auth:

    Login successful.

These credentials will remain valid until you revoke them, which you can do
with the following:

    $ snapcraft logout
    Clearing credentials for Ubuntu One SSO.
    Credentials cleared.

Now, let's upload that snap!


## Upload the snap

Run the `snapcraft upload` command passing the path to the `snap` file as an argument:

    $ snapcraft upload foo_1_amd64.snap

    Uploading foo_1_amd64.snap [==========================================] 100%
    Checking package status... |

    Application uploaded successfully (as revision 1)
    Please check out the application at: https://myapps.developer.ubuntu.com/dev/click-apps/1337/

Your snap has now been uploaded to the store, and is now undergoing an
automated review. You'll be emailed when the review has completed, at which time
you can visit the MyApps site and publish your snap! Note that if you uploaded
a new version for an already-published snap, your update will be automatically
published.
