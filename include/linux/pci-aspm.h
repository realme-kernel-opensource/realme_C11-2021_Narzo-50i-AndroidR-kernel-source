/* SPDX-License-Identifier: GPL-2.0 */
/*
 *	aspm.h
 *
 *	PCI Express ASPM defines and function prototypes
 *
 *	Copyright (C) 2007 Intel Corp.
 *		Zhang Yanmin (yanmin.zhang@intel.com)
 *		Shaohua Li (shaohua.li@intel.com)
 *
 *	For more information, please consult the following manuals (look at
 *	http://www.pcisig.com/ for how to get them):
 *
 *	PCI Express Specification
 */

#ifndef LINUX_ASPM_H
#define LINUX_ASPM_H

#include <linux/pci.h>

#define PCIE_LINK_STATE_L0S	1
#define PCIE_LINK_STATE_L1	2
#define PCIE_LINK_STATE_CLKPM	4

#ifdef CONFIG_PCIEASPM
void pcie_aspm_init_link_state(struct pci_dev *pdev);
void pcie_aspm_exit_link_state(struct pci_dev *pdev);
void pcie_aspm_pm_state_change(struct pci_dev *pdev);
void pcie_aspm_powersave_config_link(struct pci_dev *pdev);
void pci_disable_link_state(struct pci_dev *pdev, int state);
void pci_disable_link_state_locked(struct pci_dev *pdev, int state);
void pcie_no_aspm(void);
/*
 * ASPM can be disabled or enabled at runtime via
 * /sys/module/pcie_aspm/parameters/policy.
 * However, some endpoint (e.g. wcn pcie) drivers want to disabled
 * or enabled it directly. Therefore, we supply these two APIs.
 */
ssize_t sprd_pcie_aspm_set_policy(struct pci_dev *pdev, int val);
ssize_t sprd_pcie_aspm_get_policy(struct pci_dev *pdev, int *val);
#else
static inline void pcie_aspm_init_link_state(struct pci_dev *pdev)
{
}
static inline void pcie_aspm_exit_link_state(struct pci_dev *pdev)
{
}
static inline void pcie_aspm_pm_state_change(struct pci_dev *pdev)
{
}
static inline void pcie_aspm_powersave_config_link(struct pci_dev *pdev)
{
}
static inline void pci_disable_link_state(struct pci_dev *pdev, int state)
{
}
static inline void pcie_no_aspm(void)
{
}
static inline ssize_t sprd_pcie_aspm_set_policy(struct pci_dev *pdev, int val)
{
	return -EINVAL;
}
static inline ssize_t sprd_pcie_aspm_get_policy(struct pci_dev *pdev, int *val)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PCIEASPM_DEBUG /* this depends on CONFIG_PCIEASPM */
void pcie_aspm_create_sysfs_dev_files(struct pci_dev *pdev);
void pcie_aspm_remove_sysfs_dev_files(struct pci_dev *pdev);
#else
static inline void pcie_aspm_create_sysfs_dev_files(struct pci_dev *pdev)
{
}
static inline void pcie_aspm_remove_sysfs_dev_files(struct pci_dev *pdev)
{
}
#endif
#endif /* LINUX_ASPM_H */
