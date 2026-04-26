// zephyr/src/shell/shell_auth.cpp
// Shell login command and unlock state.

#include <zephyr/logging/log.h>
#include <string.h>

#include "shell_shared.hpp"
#include "http_auth.hpp"  // HDV_API_TOKEN

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static bool g_shell_unlocked = false;

namespace shell_cmds {

bool require_login(const struct shell* sh)
{
    if (!g_shell_unlocked) {
        shell_error(sh, "Locked — run 'login <token>' first");
        return false;
    }
    return true;
}

} // namespace shell_cmds

static int cmd_login(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: login <token>");
        return -EINVAL;
    }
    if (strncmp(argv[1], HDV_API_TOKEN, strlen(HDV_API_TOKEN)) == 0 &&
        strlen(argv[1]) == strlen(HDV_API_TOKEN)) {
        g_shell_unlocked = true;
        shell_print(sh, "Shell unlocked — destructive commands enabled");
    } else {
        shell_error(sh, "Invalid token");
        g_shell_unlocked = false;
        return -EACCES;
    }
    return 0;
}
SHELL_CMD_REGISTER(login, NULL, "Unlock destructive commands", cmd_login);
