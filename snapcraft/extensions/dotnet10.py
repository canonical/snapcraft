# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from overrides import override

from .dotnet import DotnetExtensionBase


class Dotnet10Extension(DotnetExtensionBase):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET 8 content snaps.
    """

    @property
    @override
    def runtime_content_snap_name(self) -> str:
        return "dotnet-runtime-100"

    @property
    @override
    def versioned_plugin_name(self) -> str:
        return "dotnet10"
