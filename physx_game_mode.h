#ifndef PHYSX_GAME_MODE_H
#define PHYSX_GAME_MODE_H

/* Observe executed native UI-mode includes, not script bodies or missing-file
   messages containing the same name. Reuse the existing incremental log read;
   this diagnostic never selects physics ownership or changes game state. */
static void physx_game_mode_log_line(const char *line)
{
    static char previous[64];
    static const char prefix[] = "ExecuteFile 'LUA/VAR_Mode___";
    const char *start, *end, *p;
    char mode[64];
    size_t length;
    if (!line) return;
    while (*line == ' ' || *line == '\t') line++;
    if (strncmp(line, prefix, sizeof(prefix) - 1)) return;
    start = line + sizeof(prefix) - 1;
    end = strstr(start, "|0'");
    if (!end || (length = (size_t)(end - start)) == 0 ||
        length >= sizeof(mode)) return;
    for (p = start; p < end; p++) {
        if (!((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
              (*p >= '0' && *p <= '9') || *p == '_')) return;
    }
    p = end + 3;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p) return;
    memcpy(mode, start, length);
    mode[length] = 0;
    if (!strcmp(previous, mode)) return;
    log_line("physics game-mode observed previous=\"%s\" native_ui_mode=\"%s\" source=\"good-bye-txx.log ExecuteFile\" note=\"timestamp is observation time; native UI mode only, independent of physics ownership\"",
        previous[0] ? previous : "unknown", mode);
    memcpy(previous, mode, length + 1);
}

#endif
