import qbs

Project {
    name: "Qbs Nil Snap"

    property bool printString: false
    PropertyOptions {
        name: "printString"
        description: "
            Set in the qbs-options property of the snapcraft.yaml

            Setting to true will print the required string for the test
            if it remains false an incorrect string is returned.
        "
        allowedValues: ["true", "false"]
    }

    CppApplication {
        name: "Snap Qbs Nil Binary"
        targetName: "qbs-nil"
        files: "*.cpp"

        cpp.cxxLanguageVersion: "c++11";
        cpp.cxxStandardLibrary: "libstdc++";
        cpp.includePaths: [ path ]

        Properties {
            condition: project.printString == true
            cpp.defines: ["PRINT_STRING"]
        }

        Group {
            qbs.install: true
            qbs.installDir: "bin"
            fileTagsFilter: product.type
        }
    }
}
