# Syntax of the snapcraft.yaml file

The `snapcraft.yaml` file is the main entry point to create a snap through
Snapcraft. The main building blocks are parts and each defined part is
independent from each other. In addition to parts, there are attributes
that define the metadata for the snap package.

What follows is a list of all the attributes the `snapcraft.yaml` file can
contain.

 * `name` (string)  
   The name of the resulting snap.
 * `version` (string)  
   The version of the resulting snap.
 * `vendor` (string)  
   Vendor in email format as defined in RFC 5322.
 * `summary` (string)  
   A 78 character long summary for the snap.
 * `description` (string)  
   The description for the snap, this can and is expected to be a longer
   explanation for the snap.
 * `config` (string)  
   Path to the runnable in snap that will be used as the Snappy config
   interface.
 * `services` (yaml subsection)  
   A set of keys representing service names with values as defined by the
   [Snappy packaging spec](https://developer.ubuntu.com/snappy/guides/packaging-format-apps/).
 * `binaries` (yaml subsection)  
   A set of keys representing binary names with values as defined by the
   [Snappy packaging spec](https://developer.ubuntu.com/snappy/guides/packaging-format-apps/).
 * `icon` (string)  
   Path to the icon that will be used for the snap.
 * `license` (yaml subsection)  
   License the snap will carry.
   * `text` (string)  
     The license text for the snap itself.
   * `accept-required` (boolean)  
     If true, license acceptance is required for the package to be activated.
     A good example for this one is the Sun JRE/JDK being bundled in a snap.
 * `framework-policy` (string)  
   A relative path to a directory containing additional policies, used if
   creating a framework and want to extend permissions to snap apps.
 * `parts` (yaml subsection)  
   A map of part names to their own part configuration. Order in the file is
   not relevant (to aid copy-and-pasting).
   * `plugin` (string)  
     Specifies the plugin name that will manage this part. Snapcraft will pass
     to it all the other user-specified part options. If plugin is not
     defined, [the wiki](https://wiki.ubuntu.com/Snappy/Parts) will be
     searched for the part, the local values defined in the part will be used
     to compose the final part.
   * `after` (list of strings)  
     Specifies any parts that should be built before this part is. This is
     mostly useful when a part needs a library or build tool built by another
     part. If the part defined in after is not defined locally, the part will
     be searched for in [the wiki](https://wiki.ubuntu.com/Snappy/Parts).
     *If a part is supposed to run after another, the prerequisite part will
     be staged before the dependent part starts its lifecycle.*
   * `stage-packages` (list of strings)  
     A list of Ubuntu packages to use that would support the part creation.
   * `filesets` (yaml subsection)  
     A dictionary with filesets, the key being a recognizable user defined
     string and its value a list of strings of files to be included or
     excluded. Globbing is achieved with `*` for either inclusions or
     exclusion. Exclusions are denoted by a `-`. Globbing is computed from
     the private sections of the part.
   * `organize` (yaml subsection)  
     A dictionary exposing replacements, the key is the internal name whilst
     the value the exposed name, filesets will refer to the exposed named
     applied after organization is applied.
   * `stage` (list of strings)
     A list of files from a part's installation to expose in stage. Rules
     applying to the list here are the same as those of filesets. Referencing
     of fileset keys is done with a $ prefixing the fileset key, which will
     expand with the value of such key.
   * `snap` (list of strings)  
     A list of files from a part's installation to expose in snap. Rules
     applying to the list here are the same as those of filesets. Referencing
     of fileset keys is done with a `$` prefixing the fileset key, which will
     expand with the value of such key.

The `snapcraft.yaml` in any project is validated to be compliant to these
keywords, if there is any missing expected component or invalid value,
`snapcraft` will exit with an error.
