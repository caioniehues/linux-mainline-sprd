/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PANEL_GENERIC_DSI_H
#define _LINUX_PANEL_GENERIC_DSI_H

#include <linux/kconfig.h>
#include <linux/notifier.h>

#define GENERIC_PANEL_EVENT_PREPARED	1
#define GENERIC_PANEL_EVENT_UNPREPARED	2

#if IS_ENABLED(CONFIG_DRM_PANEL_GENERIC_DSI)
int generic_panel_register_notifier(struct notifier_block *nb);
int generic_panel_unregister_notifier(struct notifier_block *nb);
#else
static inline int generic_panel_register_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int generic_panel_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}
#endif

#endif /* _LINUX_PANEL_GENERIC_DSI_H */
