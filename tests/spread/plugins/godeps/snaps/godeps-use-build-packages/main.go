// -*- Mode: Go; indent-tabs-mode: t -*-

/*
 * Copyright (C) 2016 Canonical Ltd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

package main

import (
    "fmt"
    "os"
    "strconv"
    "golang.org/x/crypto/bcrypt"
)

func main() {
    if len(os.Args) != 4 {
        fmt.Println("Usage:")
        fmt.Printf("\t%s hash <cost> <password>\n", os.Args[0])
        fmt.Printf("\t%s check <hashed password> <password>\n", os.Args[0])
        os.Exit(1)
    }

    switch os.Args[1] {
    case "hash":
        hash()
    case "check":
        check()
    default:
        fmt.Printf("Unknown command: %q\n", os.Args[1])
        os.Exit(1)
    }
}

func hash() {
    password := []byte(os.Args[3])
    cost, err := strconv.Atoi(os.Args[2])
    if err != nil {
        fmt.Printf("ERROR: Unable to convert %q to a number\n", os.Args[1])
        os.Exit(1)
    }

    hash, err := bcrypt.GenerateFromPassword(password, cost)
    if err != nil {
        fmt.Printf("ERROR: Unable to hash %q with the cost of %d\n", password, cost)
        os.Exit(1)
    }

    fmt.Println(string(hash))
}

func check() {
    hashed_password := []byte(os.Args[2])
    password := []byte(os.Args[3])

    err := bcrypt.CompareHashAndPassword(hashed_password, password)
    if err != nil {
        fmt.Println("Not equal")
        os.Exit(1)
    }

    fmt.Println("Equal")
}
