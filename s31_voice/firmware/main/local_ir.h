#pragma once

#include <stdbool.h>
#include "esp_err.h"

/* Experimental two-command profile. Disabled until TX wiring is configured.
 * All calls except stage_commands run on main/action task, never SR callbacks. */
esp_err_t local_ir_init(void);
bool local_ir_profile_enabled(void);
void local_ir_stage_commands(void);
/* true = this profile owns the command, even if transmission failed.
 * Caller must NEVER retry an owned command through the server. */
bool local_ir_execute(const char *text, esp_err_t *result);
