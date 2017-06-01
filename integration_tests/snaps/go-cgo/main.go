// -*- Mode: Go; indent-tabs-mode: t -*-

/*
 * Copyright (C) 2017 Canonical Ltd
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

/*
#cgo pkg-config: glib-2.0
#cgo LDFLAGS: -lm
#include <math.h>
#include <glib.h>

*/
import "C"

import (
	"fmt"
	"os/user"
)

func main() {
	u, err := user.Current()
	if err != nil {
		panic(err)
	}
	fmt.Println("user according to go:", u.Username)
	fmt.Println("user according to glib:", C.GoString((*C.char)(C.g_get_user_name())))
	fmt.Println("more or less 1:", C.sin(3.14/2))
}
