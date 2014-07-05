/*
 * Copyright (C) ST-Ericsson SA 2011
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/pasr.h>

#include "helper.h"

enum pasr_state {
	PASR_REFRESH,
	PASR_NO_REFRESH,
};

struct pasr_fw {
	struct pasr_map *map;
};

static struct pasr_fw pasr;

void pasr_update_mask(struct pasr_section *section, enum pasr_state state)
{
	struct pasr_die *die = section->die;
	phys_addr_t addr = section->start - die->start;
	u8 bit = addr >> PASR_SECTION_SZ_BITS;

	if (state == PASR_REFRESH)
		clear_bit(bit, &die->mem_reg);
	else
		set_bit(bit, &die->mem_reg);

	pr_debug("%s(): %s refresh section %#x. Die%d mem_reg = %#08lx\n"
			, __func__, state == PASR_REFRESH ? "Start" : "Stop"
			, section->start, die->idx, die->mem_reg);

	if (die->apply_mask)
		die->apply_mask(&die->mem_reg, die->cookie);

	return;
}

void pasr_put(phys_addr_t paddr, unsigned long size)
{
	struct pasr_section *s;
	unsigned long cur_sz;
	unsigned long flags = 0;

	if (!pasr.map)
		goto out;

	do {
		s = pasr_addr2section(pasr.map, paddr);
		if (!s)
			goto out;

		cur_sz = ((paddr + size) < (s->start + PASR_SECTION_SZ)) ?
			size : s->start + PASR_SECTION_SZ - paddr;

		if (s->lock)
			spin_lock_irqsave(s->lock, flags);

		s->free_size += cur_sz;
		BUG_ON(s->free_size > PASR_SECTION_SZ);

		if (s->free_size < PASR_SECTION_SZ)
			goto unlock;

		if (!s->pair)
			pasr_update_mask(s, PASR_NO_REFRESH);
		else if (s->pair->free_size == PASR_SECTION_SZ) {
				pasr_update_mask(s, PASR_NO_REFRESH);
				pasr_update_mask(s->pair, PASR_NO_REFRESH);
		}
unlock:
		if (s->lock)
			spin_unlock_irqrestore(s->lock, flags);

		paddr += cur_sz;
		size -= cur_sz;
	} while (size);

out:
	return;
}

void pasr_get(phys_addr_t paddr, unsigned long size)
{
	unsigned long flags = 0;
	unsigned long cur_sz;
	struct pasr_section *s;

	if (!pasr.map)
		goto out;

	do {
		s = pasr_addr2section(pasr.map, paddr);
		if (!s)
			goto out;

		cur_sz = ((paddr + size) < (s->start + PASR_SECTION_SZ)) ?
			size : s->start + PASR_SECTION_SZ - paddr;

		if (s->lock)
			spin_lock_irqsave(s->lock, flags);

		if (s->free_size < PASR_SECTION_SZ)
			goto unlock;

		if (!s->pair)
			pasr_update_mask(s, PASR_REFRESH);
		else if (s->pair->free_size == PASR_SECTION_SZ) {
				pasr_update_mask(s, PASR_REFRESH);
				pasr_update_mask(s->pair, PASR_REFRESH);
		}
unlock:
		BUG_ON(cur_sz > s->free_size);
		s->free_size -= cur_sz;

		if (s->lock)
			spin_unlock_irqrestore(s->lock, flags);

		paddr += cur_sz;
		size -= cur_sz;
	} while (size);

out:
	return;
}

int pasr_register_mask_function(phys_addr_t addr, void *function, void *cookie)
{
	struct pasr_die *die = pasr_addr2die(pasr.map, addr);

	if (!die) {
		pr_err("%s: No DDR die corresponding to address 0x%08x\n",
				__func__, addr);
		return -EINVAL;
	}

	if (addr != die->start)
		pr_warning("%s: Addresses mismatch (Die = 0x%08x, addr = 0x%08x\n"
				, __func__, die->start, addr);

	die->cookie = cookie;
	die->apply_mask = function;

	die->apply_mask(&die->mem_reg, die->cookie);

	return 0;
}

int __init pasr_init_core(struct pasr_map *map)
{
	pasr.map = map;
	return 0;
}

