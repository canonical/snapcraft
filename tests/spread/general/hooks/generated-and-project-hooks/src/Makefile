GO=go
GOFLAGS=-ldflags="-s -w"

build:
	$(GO) build $(GOFLAGS) -o configure configure.go
	$(GO) build $(GOFLAGS) -o install install.go

tidy:
	go mod tidy -compat=1.20

clean:
	rm -f configure install
