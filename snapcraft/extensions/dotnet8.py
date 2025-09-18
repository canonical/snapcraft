# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from .dotnet import DotnetExtensionBase


class Dotnet8Extension(DotnetExtensionBase):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET 8 content snaps.
    """

    @property
    def runtime_content_snap_name(self) -> str:
        return "dotnet-runtime-80"
