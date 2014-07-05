/*
 * Copyright (C) 2012 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#define NEEDS_STDIO  /* for snprintf */

#include "middle/proactive_calculations.h"
#include "common/essl_system.h"
#include "common/essl_list.h"
#include "common/ptrdict.h"
#include "common/ptrset.h"
#include "common/essl_node.h"
#include "common/basic_block.h"
#include "common/essl_type.h"
#include "common/symbol_table.h"
#include "middle/dominator.h"

/**
 * This file includes implementation of the pilot shading optimization.
 * The compiler identifies chains of operations dependent only on run-time constants (e.g. uniform varibales)
 * and moves them to a separate pilot shader (up to 4). 
 * Afterwards driver executes these pilot shaders before main jobs
 */

/* rt stands for run-time */

/* just a list of nodes */
typedef struct run_time_nodes_list
{
	struct run_time_nodes_list *next; /* Next element in the list */
	node *node; /* rt constant node*/
} run_time_nodes_list;
ASSUME_ALIASING(run_time_nodes_list, generic_list);

/**
 * structure that keeps all necessary infor about an rt node
 */
typedef struct run_time_constant_node 
{
	struct run_time_constant_node *next;	/* Next element in the list */
	node *node;								/* an rt constant node to be moved to a pilot shader (with all necessary predecessors)*/
	node *orig_rt_node;						/* an originally found rt constant node */
	run_time_nodes_list *prev_rt_nodes;		/* list of rt nodes */
	basic_block *orig_rt_bb;				/* basic block where original rt node is located */
	int weight;								/* rt node weight + weights of all dependent rt constant calculations */
	symbol *orig_func;						/* function where rt node was found */
} run_time_constant_node;
ASSUME_ALIASING(run_time_constant_node, generic_list);

/**
 * Optimization context
 */
typedef struct pilot_calculations_context
{
	mempool *pool;
	mempool *temp_pool;
	target_descriptor *desc;
	translation_unit *tu;
	typestorage_context *ts_ctx;
	control_flow_graph *cfg;
	symbol *func;
	ptrdict node_succs;					/* dictionry of nodes' successors */
	ptrdict rt_const_nodes;
	ptrset visited;
	ptrset hoist_points;				/* nodes to move to a pilot shader */
	ptrdict copied;						/* nodes that have been cloned during a pilot shader creation */
	run_time_constant_node *rtc_nodes;
	essl_bool applied;					/* flag to set if the optimization was applied */
}pilot_calculations_context;

/* A list of node successors */
typedef struct node_succs_list {
	struct node_succs_list *next; 	/* Next element in the list */
	node *succ; 					/* successor */
} node_succs_list;
ASSUME_ALIASING(node_succs_list, generic_list);

static memerr find_last_fully_const_succ(pilot_calculations_context *ctx, node *n, run_time_constant_node *rtc_elem, int *calc_weight_p);
#define MIN_ALLOWED_WEIGHT 5
/**
 * weight function for pilot calculations
 */
static int get_node_pilot_weight(node *op)
{
	switch (op->hdr.kind) {
	case EXPR_KIND_PHI:
	case EXPR_KIND_DONT_CARE:
	case EXPR_KIND_TRANSFER:
	case EXPR_KIND_DEPEND:
	case EXPR_KIND_VECTOR_COMBINE:
	case EXPR_KIND_CONSTANT:
	case EXPR_KIND_VARIABLE_REFERENCE:
		return 0;
	case EXPR_KIND_UNARY:
		switch(op->expr.operation)
		{
		case EXPR_OP_SWIZZLE:
			return 0;
		default:
			return 1;
		}
	case EXPR_KIND_BUILTIN_FUNCTION_CALL:
		if(_essl_node_is_texture_operation(op))
		{
			return 10;
		}
		switch(op->expr.operation)
		{
		case EXPR_OP_FUN_CLAMP:
		case EXPR_OP_FUN_ABS:
		case EXPR_OP_FUN_TRUNC:
			return 0;
		default:
			return 1;
		}

	default:
		return 1;
	}
}


/**
 * Analyze whether the node is run time constant
 *
 * @param ctx	optimization context
 * @param op	node to analyze
 * @param weight	node weight to return
 *
 * @returns ESSL_TRUE if node is run time constant, ESSL_FALSE otherwise
 */
static essl_bool is_node_inputs_rt_constant(pilot_calculations_context *ctx, node *op, int *weight, int *child_weight_p)
{
	unsigned i;
	essl_bool res = ESSL_TRUE;
	int child_weight = 0;

	/* get the node weight */
	if(weight != NULL)
	{
		*weight += get_node_pilot_weight(op);
	}

	if(child_weight_p != NULL)
	{
		*child_weight_p += get_node_pilot_weight(op);
	}


	/* check if the node has been analyzed */
	if((_essl_ptrdict_has_key(&ctx->rt_const_nodes, op)) == ESSL_TRUE)
	{
		return ESSL_TRUE;
	}
	
	/* only uniform loads are conisdered to be run time constant */
	if(op->hdr.kind == EXPR_KIND_LOAD)
	{
		if(op->expr.u.load_store.address_space == ADDRESS_SPACE_UNIFORM)
		{
			node *address;
			/* if addresing expression consists only from a variable reference it's definitely run-time constant */
			ESSL_CHECK(address = GET_CHILD(op, 0));
			if(address->hdr.kind == EXPR_KIND_VARIABLE_REFERENCE)
			{
				return ESSL_TRUE;
			}
		}else
		{
			return ESSL_FALSE;
		}
	}

	/* always constant */
	if(op->hdr.kind == EXPR_KIND_CONSTANT)
	{
		return ESSL_TRUE;
	}
	
	/* it's too dififcult to process phi-nodes as it may involve  additional control flow analysis
	 * jut returning false for now
	 */
	if(op->hdr.kind == EXPR_KIND_PHI)
	{
		return ESSL_FALSE;
	}

	/* no way */
	if(op->hdr.kind == EXPR_KIND_FUNCTION_CALL)
	{
		return ESSL_FALSE;
	}

	/* check all input nodes */
	for(i = 0; i < GET_N_CHILDREN(op); i++)
	{
		node *child = GET_CHILD(op, i);
		res &= is_node_inputs_rt_constant(ctx, child, weight, &child_weight);
	}
	
	/* hash run time constants */
	if(res == ESSL_TRUE)
	{
		ESSL_CHECK(_essl_ptrdict_insert(&ctx->rt_const_nodes, op, (void *)(long)child_weight));
	}
	return res;
}

/**
 * Adding a successor to the list
 *
 * @param ctx	optimization context
 * @param n		node
 * @param succ	successor of n
 *
 * @returns MEM_OK if all allocation were correct
 */
static memerr add_succs_to_list(pilot_calculations_context *ctx, node *n, node *succ)
{
	node_succs_list *succs_list;
	node_succs_list *sl;
	succs_list = _essl_ptrdict_lookup(&ctx->node_succs, n);
	ESSL_CHECK(sl = LIST_NEW(ctx->temp_pool, node_succs_list));
	sl->succ = succ;
	LIST_INSERT_BACK(&succs_list, sl);
	ESSL_CHECK(_essl_ptrdict_insert(&ctx->node_succs, n, succs_list));

	return MEM_OK;
}

/**
 * Collecting successors for the node
 *
 * @param ctx	optimization context
 * @param n		node
 *
 * @returns MEM_OK if all allocation were correct
 */
static memerr collect_successors_for_node(pilot_calculations_context *ctx, node *n)
{
	unsigned i;

	/* check if the node was processed */
	if((_essl_ptrset_has(&ctx->visited, n)) == ESSL_TRUE)
	{
		return MEM_OK;
	}
	ESSL_CHECK(_essl_ptrset_insert(&ctx->visited, n));
	
	if(n->hdr.kind == EXPR_KIND_PHI)
	{
		/* process phi-node */
		phi_source *src;
		for(src = n->expr.u.phi.sources; src != 0; src = src->next)
		{
			ESSL_CHECK(add_succs_to_list(ctx, src->source, n));
			ESSL_CHECK(collect_successors_for_node(ctx, src->source));
		}
	} else 
	{
		/* process regular node */
		for(i = 0; i < GET_N_CHILDREN(n); ++i)
		{
			node *child = GET_CHILD(n, i);
			if(child->hdr.kind != EXPR_KIND_PHI)
			{
				ESSL_CHECK(add_succs_to_list(ctx, child, n));
				ESSL_CHECK(collect_successors_for_node(ctx, child));
			}
		}
	}	

	return MEM_OK;
}

/**
 * Collecting successors for nodes in the basic block
 *
 * @param ctx	optimization context
 * @param b		basic block
 *
 * @returns MEM_OK if all allocation were correct
 */

static memerr collect_successors(pilot_calculations_context *ctx, basic_block *b)
{
	phi_list *phi;
	control_dependent_operation *c_op;
	if(b->source)
	{
		ESSL_CHECK(collect_successors_for_node(ctx, b->source));
	}
	for(c_op = b->control_dependent_ops; c_op != 0; c_op = c_op->next)
	{
		node *n = c_op->op;
		ESSL_CHECK(collect_successors_for_node(ctx, n));
	}

	for(phi = b->phi_nodes; phi != 0; phi = phi->next)
	{		
		ESSL_CHECK(collect_successors_for_node(ctx, phi->phi_node));
	}

	return MEM_OK;
}

/**
 *	Find successors for the node if not all of them are rt constant
 *
 *	@param ctx	optimization context
 *	@param n	node
 *	@param has_succs	returns if the node has successors
 *
 *	@returns list of successors if at least one of them is not rt constant
 */
static node_succs_list * find_rt_const_succs(pilot_calculations_context *ctx, node *n, essl_bool *has_succs)
{
	essl_bool all_succs_const = ESSL_TRUE;
	node_succs_list *succs_list;
	node_succs_list *sl;
	node *succ;
	essl_bool is_const;
	*has_succs = ESSL_TRUE;
	succs_list = _essl_ptrdict_lookup(&ctx->node_succs, n);
	if(succs_list == NULL && n->hdr.kind != EXPR_KIND_STORE)
	{
		*has_succs = ESSL_FALSE;
		return NULL;
	}
	for(sl = succs_list; sl != NULL; sl = sl->next)
	{
		int weight = 0;
		succ = sl->succ;
		is_const = is_node_inputs_rt_constant(ctx, succ, &weight, NULL);
		if(!is_const)
		{
			all_succs_const = ESSL_FALSE;
		}
	}

	if(all_succs_const == ESSL_FALSE)
	{
		return succs_list;
	}
	

	return NULL;
}

/**
 * comparing 2 elements of list by weight
 *
 */
static int compare_rtc_nodes_by_weight( generic_list *va, generic_list *vb)
{
	run_time_constant_node *a = (run_time_constant_node*)va;
	run_time_constant_node *b = (run_time_constant_node*)vb;

	return b->weight - a->weight;
}

/**
 * Check if the operation is used in addressing expressions
 */
static essl_bool is_addressing_op(node *n)
{
	if((n->hdr.kind == EXPR_KIND_BINARY && n->expr.operation == EXPR_OP_INDEX) ||
		(n->hdr.kind == EXPR_KIND_UNARY && n->expr.operation == EXPR_OP_MEMBER) ||
		n->hdr.kind == EXPR_KIND_VARIABLE_REFERENCE)
	{
		return ESSL_TRUE;
	}

	return ESSL_FALSE;
}

static memerr add_to_hoist_points(pilot_calculations_context *ctx, 
						node *n,
						run_time_constant_node *rtc_elem,
						node *succ, 
						int weight,
						essl_bool is_const, 
						int *calc_weight,
						essl_bool *prev_node_set)
{
	if(is_const)
	{
		int tmp_weight;
		run_time_nodes_list * prev_rt_node_elem;
		if(succ == ctx->cfg->exit_block->source)
		{
			/* if successor is the source for the exit then we should not remove it */
			*calc_weight = weight;
			return MEM_OK;
		}


		ESSL_CHECK(prev_rt_node_elem = LIST_NEW(ctx->temp_pool, run_time_nodes_list));
		prev_rt_node_elem->node = rtc_elem->node;
		LIST_INSERT_FRONT(&rtc_elem->prev_rt_nodes, prev_rt_node_elem);
		rtc_elem->node = succ;
		(*calc_weight) += weight;
		if(succ != n)
		{
			ESSL_CHECK(find_last_fully_const_succ(ctx, succ, rtc_elem, &tmp_weight));
			(*calc_weight) += tmp_weight;
		}
	}else
	{
		/* not constant */
		if(is_addressing_op(n))
		{
			run_time_nodes_list *prev_rt_node_elem;
			*prev_node_set = ESSL_TRUE;
			/* addressing operations are tightly connected with their successors and can't moved to another func without a successor */
			prev_rt_node_elem = rtc_elem->prev_rt_nodes;
			rtc_elem->node = prev_rt_node_elem->node; /* last added element */
			prev_rt_node_elem = prev_rt_node_elem->next;
			while(is_addressing_op(rtc_elem->node) && prev_rt_node_elem!= NULL)
			{
				rtc_elem->node = prev_rt_node_elem->node;
				prev_rt_node_elem = prev_rt_node_elem->next;
			}
		}
	}
	ESSL_CHECK(_essl_ptrset_insert(&ctx->hoist_points, rtc_elem->node));

	return MEM_OK;

}
/**
 * For the node find the last (in depth) successor which is rt constant
 *
 * @param ctx	optimization context
 * @param n		node
 * @param rtc_elem	element of run time constant nodes list
 *
 * @returns weight
 */
static memerr find_last_fully_const_succ(pilot_calculations_context *ctx, node *n, run_time_constant_node *rtc_elem, int *calc_weight_p)
{
	int calc_weight = 0;
	node_succs_list *succs_list;
	node_succs_list *sl;
	node *succ;
	essl_bool is_const;
	essl_bool prev_node_set = ESSL_FALSE;
	succs_list = _essl_ptrdict_lookup(&ctx->node_succs, n);
	/* traversing all successors */
	if(succs_list == NULL)
	{
		int weight = 0;
		if(n->hdr.kind == EXPR_KIND_STORE)
		{
			node *pred = GET_CHILD(n, 1);
			ESSL_CHECK(pred);
			is_const = is_node_inputs_rt_constant(ctx, pred, &weight, NULL);
			ESSL_CHECK(add_to_hoist_points(ctx, pred, rtc_elem, pred, weight, is_const, &calc_weight, &prev_node_set));
		}
	}else
	{
		for(sl = succs_list; sl != NULL; sl = sl->next)
		{
			int weight = 0;
			succ = sl->succ;
			if(succ->hdr.kind == EXPR_KIND_PHI)
			{
				if(succ->expr.u.phi.block->postorder_visit_number < rtc_elem->orig_rt_bb->postorder_visit_number)
				{
					/* loop structure: don't work with loops as it means that we should modife control flow*/
					rtc_elem->orig_rt_node = NULL;
					_essl_ptrset_clear(&ctx->hoist_points);
					return -1;
				}
				is_const = ESSL_FALSE;
			}else if(succ->hdr.kind == EXPR_KIND_STORE)
			{
				is_const = ESSL_FALSE;
			}else
			{
				is_const = is_node_inputs_rt_constant(ctx, succ, &weight, NULL);
			}

			ESSL_CHECK(add_to_hoist_points(ctx, n, rtc_elem, succ, weight, is_const, &calc_weight, &prev_node_set));
		}
	}

	/* if there are more than 2 successors to move we'll just drop, 
	 * don't know how to handle it now */	
	if(_essl_ptrset_size(&ctx->hoist_points) > 1)
	{
		run_time_nodes_list * elem;

		if(!prev_node_set)
		{
			rtc_elem->node = n;
		}
		for(elem = rtc_elem->prev_rt_nodes; elem != NULL; elem = elem->next)
		{
			if(elem->node == rtc_elem->node)
			{
				rtc_elem->prev_rt_nodes = elem;
			}
		}
		calc_weight = 0;
	}
	_essl_ptrset_clear(&ctx->hoist_points);

	*calc_weight_p = calc_weight;

	return MEM_OK;
}

/**
 * For the node find the last (in depth) successor which is rt constant
 *
 * @param ctx optimization context
 */
static memerr find_last_point_for_hoisting_out(pilot_calculations_context *ctx)
{
	run_time_constant_node *elem;
	run_time_constant_node *elem_n;
	run_time_constant_node *prev_elem;
	run_time_constant_node *telem, *telem_n;
	int weight;

	prev_elem = NULL;
	for(elem = ctx->rtc_nodes; elem != NULL; elem = elem_n)
	{
		/* for each  rt constant candidate*/
		int tmp_weight;
		node_succs_list *succs_list;
		elem_n = elem->next;
		weight = get_node_pilot_weight(elem->node);
		ESSL_CHECK(_essl_ptrdict_clear(&ctx->rt_const_nodes));
		ESSL_CHECK(find_last_fully_const_succ(ctx, elem->node, elem, &tmp_weight));
		elem->weight = weight + tmp_weight;
		succs_list = _essl_ptrdict_lookup(&ctx->node_succs, elem->node);
		if(succs_list == NULL || elem->orig_rt_node == NULL)
		{
			/* either no succs - dead code or
			 * we found a loop
			 * anyway it's not suitable to move to a pilot shader
			 */
			if(prev_elem != NULL)
			{
				prev_elem->next = elem_n;
			}
			if(elem == ctx->rtc_nodes)
			{
				ctx->rtc_nodes = elem_n;
			}
		}
		prev_elem = elem;
	}

	ctx->rtc_nodes = LIST_SORT(ctx->rtc_nodes, compare_rtc_nodes_by_weight, run_time_constant_node);

	elem = ctx->rtc_nodes;
	/* it is possible that for different candidates we found 
	 * the same successor which should be moved
	 * let's remove duplicates
	 * */
	while(elem != NULL) /* removing duplicating hoisting points */
	{
		prev_elem = elem;
		for(telem = elem->next; telem != NULL; telem = telem_n)
		{
			telem_n = telem->next;
			if(elem->node == telem->node)
			{
				prev_elem->next = telem_n;
			}
			prev_elem = elem;
		}
		elem = elem->next;
	}

	return MEM_OK;

}

/**
 * Control dependent operations that we moved to a pilot shaders should be removed from structures in the original function
 *
 * @param ctx optimization context
 * @param orig_func original function
 * @param pilot_func pilot shader
 * @param n node to analyze
 *
 * @returns MEM_OK if all allocation were correct
 */
static memerr fix_control_dependent_preds(pilot_calculations_context *ctx, symbol *orig_func, symbol *pilot_func, node *n)
{
	unsigned i;

	if(n->hdr.is_control_dependent)
	{
		control_dependent_operation *p;
		basic_block *bb = pilot_func->control_flow_graph->exit_block;
		p = _essl_ptrdict_lookup(&orig_func->control_flow_graph->control_dependence, n);
		if(p == NULL)
		{
			/* it's strange that a control dependent operation was used twice */
			assert(_essl_ptrdict_lookup(&pilot_func->control_flow_graph->control_dependence, n) != NULL);
			return MEM_OK;
		}
		_essl_ptrdict_remove(&orig_func->control_flow_graph->control_dependence, n);
		_essl_remove_control_dependent_op_node(&p->block->control_dependent_ops, n);
		p->block = bb;
		p->next = NULL;
		ESSL_CHECK(_essl_ptrdict_insert(&pilot_func->control_flow_graph->control_dependence, n, p));
		LIST_INSERT_FRONT(&bb->control_dependent_ops, p);
	}
	for(i = 0; i < GET_N_CHILDREN(n); i++)
	{
		node* n_child;
		ESSL_CHECK(n_child = GET_CHILD(n, i));
		fix_control_dependent_preds(ctx, orig_func, pilot_func, n_child);
	}

	return MEM_OK;

}

/**
 * Copy a rt constant node
 */
static node * copy_rtc_node(pilot_calculations_context *ctx, node *n)
{
	unsigned i;
	node *res;
	res = _essl_ptrdict_lookup(&ctx->copied, n);
	if(res != NULL)
	{
		return res;
	}

	ESSL_CHECK(res = _essl_clone_node(ctx->pool, n));
	if(n->hdr.is_control_dependent)
	{
		_essl_clone_control_dependent_op(n, res, ctx->cfg, ctx->pool);
	}
	for(i = 0; i < GET_N_CHILDREN(n); i++)
	{
		node *pred, *copy_pred;
		pred = GET_CHILD(n, i);
		copy_pred = copy_rtc_node(ctx, pred);
		SET_CHILD(res, i, copy_pred);
	}
	ESSL_CHECK(_essl_ptrdict_insert(&ctx->copied, n, res));
	return res;
}

/**
 * Move all operations that operate on run time constant input to a pilot shader
 *
 * @param ctx optimization context
 * @param pilot_func pilot shader function where to move
 * @param rt_node run time constant node to move
 * @param num number of the created pilot shader function
 *
 * @returns MEM_OK if all allocations were correct
 */
static memerr move_calculations_to_pilot_shader(pilot_calculations_context *ctx, symbol *pilot_func, run_time_constant_node *rt_node, unsigned num)
{
	symbol *sym;
	qualifier_set qual;
	node *load;
	node *new_input;
	node *address;
	node *succ;
	node_succs_list *succs_list;
	node_succs_list *sl;
	symbol_list *sym_list;
	node *copy;
	node *out;
	unsigned vec_size;
	char uni_name[30];
	string sym_name;

	_essl_init_qualifier_set(&qual);
	if(ctx->desc->kind == TARGET_FRAGMENT_SHADER)
	{
		const type_specifier *vec4;
		snprintf(uni_name, 30, "?__gl_mali_pilot_uniform_%d", num);
		sym_name = _essl_cstring_to_string(ctx->pool, uni_name);
		ESSL_CHECK(sym_name.ptr);

		qual.variable = VAR_QUAL_UNIFORM;
		qual.precision = PREC_MEDIUM;

		/* always 4 elements because in HW these uniforms should have a stride of pow of 2 */
		ESSL_CHECK(vec4 = _essl_get_type_with_size(ctx->ts_ctx, TYPE_FLOAT, 4, SIZE_FP16));

		/* uniform to substitute hoisted out calculations */
		ESSL_CHECK(sym = _essl_new_variable_symbol(ctx->pool, sym_name, vec4, qual, SCOPE_GLOBAL, ADDRESS_SPACE_UNIFORM, UNKNOWN_SOURCE_OFFSET));
		sym->opt.is_indexed_statically = ESSL_FALSE;
		sym->opt.pilot.proactive_uniform_num = num;
		ESSL_CHECK(sym_list = LIST_NEW(ctx->pool, symbol_list));
		sym_list->sym = sym;
		LIST_INSERT_BACK(ctx->tu->uniforms, sym_list);

		/* creating a load for the pilot uniform */
		ESSL_CHECK(address = _essl_new_variable_reference_expression(ctx->pool, sym));
		_essl_ensure_compatible_node(address, rt_node->node);

		ESSL_CHECK(load = _essl_new_load_expression(ctx->pool, ADDRESS_SPACE_UNIFORM, address));
		_essl_ensure_compatible_node(load, rt_node->node);
		load->hdr.type = vec4;

		/* taking only necessary number of components (pilot shader always writes out 4 component) */
		ESSL_CHECK(new_input = _essl_new_unary_expression(ctx->pool, EXPR_OP_SWIZZLE, load));
		_essl_ensure_compatible_node(new_input, rt_node->node);
		new_input->expr.u.swizzle = _essl_create_identity_swizzle(GET_NODE_VEC_SIZE(rt_node->node));
	}else
	{
		const type_specifier *inp_type;

		snprintf(uni_name, 30, "?__gl_mali_pilot_gp_res_%d", num);
		sym_name = _essl_cstring_to_string(ctx->pool, uni_name);
		ESSL_CHECK(sym_name.ptr);

		assert(ctx->desc->kind == TARGET_VERTEX_SHADER);
		qual.variable = VAR_QUAL_PERSISTENT;
		qual.precision = PREC_HIGH;

		inp_type = rt_node->node->hdr.type;

		/* variable to substitute hoisted out calculations */
		ESSL_CHECK(sym = _essl_new_variable_symbol(ctx->pool, sym_name, inp_type, qual, SCOPE_GLOBAL, ADDRESS_SPACE_GLOBAL, UNKNOWN_SOURCE_OFFSET));
		sym->is_persistent_variable = ESSL_TRUE;
		sym->opt.is_indexed_statically = ESSL_FALSE;
		sym->opt.pilot.proactive_uniform_num = num;
		ESSL_CHECK(sym_list = LIST_NEW(ctx->pool, symbol_list));
		sym_list->sym = sym;
		LIST_INSERT_BACK(ctx->tu->globals, sym_list);

		/* creating a load for the pilot variable */
		ESSL_CHECK(address = _essl_new_variable_reference_expression(ctx->pool, sym));
		_essl_ensure_compatible_node(address, rt_node->node);

		ESSL_CHECK(load = _essl_new_load_expression(ctx->pool, ADDRESS_SPACE_GLOBAL, address));
		_essl_ensure_compatible_node(load, rt_node->node);

		new_input = load;
	}

	/* change uses of the hoisted out instruction
	 * they will use the pilot uniform value */
	succs_list = _essl_ptrdict_lookup(&ctx->node_succs, rt_node->node);
	for(sl = succs_list; sl != NULL; sl = sl->next)
	{
		unsigned i;
		succ = sl->succ;
		if(succ->hdr.kind == EXPR_KIND_PHI)
		{
			phi_source *src;
			for(src = succ->expr.u.phi.sources; src != 0; src = src->next)
			{
				if(src->source == rt_node->node)
				{
					src->source = new_input;
				}
			}

		}else
		{
			for(i = 0; i < GET_N_CHILDREN(succ); i++)
			{
				if(GET_CHILD(succ, i) == rt_node->node)
				{
					SET_CHILD(succ, i, new_input);
				}
			}
		}
	}

	/* copy all operations that will be moved (it is done in order to simplify theirs moving to the pilot shader) */
	ESSL_CHECK(copy = copy_rtc_node(ctx, rt_node->node));
	out = copy;
	vec_size = GET_NODE_VEC_SIZE(copy);
	if(ctx->desc->kind == TARGET_FRAGMENT_SHADER)
	{
		/* determine real vector size of the pilot shader result */
		if(copy->hdr.kind == EXPR_KIND_UNARY && copy->expr.operation == EXPR_OP_SWIZZLE)
		{
			unsigned i;
			unsigned n_vec_size = vec_size;
			for(i = 0; i < vec_size; i++)
			{
				if(copy->expr.u.swizzle.indices[i] < 0)
				{
					n_vec_size--;
				}
			}
			vec_size = n_vec_size;
		}
		if(vec_size < 4)
		{
			/* if it less than 4 we need to create a vector combine with zeros in
			 * order to please remove dead code optimization */
			node *zero;
			scalar_type mali200_val;
			unsigned i;
			node *swz;

			mali200_val = ctx->desc->float_to_scalar(0);
			ESSL_CHECK(zero = _essl_new_constant_expression(ctx->pool, 4));
			for(i = 0; i < 4; i++)
			{
				zero->expr.u.value[i] = mali200_val;
			}
			ESSL_CHECK(zero->hdr.type = _essl_get_type_with_given_vec_size(ctx->ts_ctx, copy->hdr.type, 4));

			ESSL_CHECK(swz = _essl_new_unary_expression(ctx->pool, EXPR_OP_SWIZZLE, copy));
			_essl_ensure_compatible_node(swz, zero);
			swz->expr.u.swizzle = _essl_create_identity_swizzle(vec_size);

			ESSL_CHECK(out = _essl_new_vector_combine_expression(ctx->pool, 2));
			_essl_ensure_compatible_node(out, zero);
			SET_CHILD(out, 0, swz);
			SET_CHILD(out, 1, zero);
			for(i = 0; i < vec_size; i++)
			{
				out->expr.u.combiner.mask[i] = 0;
			}
			for(; i < 4; i++)
			{
				out->expr.u.combiner.mask[i] = 1;
			}
			if(copy->hdr.kind == EXPR_KIND_UNARY && copy->expr.operation == EXPR_OP_SWIZZLE)
			{
				for(i = 0; i < vec_size; i++)
				{
					if(copy->expr.u.swizzle.indices[i] < 0)
					{
						out->expr.u.combiner.mask[i] = 1;
					}
				}
			}
		}else
		{
			out = copy;
		}

		/* set pilot shader body */
		pilot_func->body = out;
		pilot_func->control_flow_graph->exit_block->source = out;
	}else
	{
		node *store;
		basic_block *bb = pilot_func->control_flow_graph->exit_block;
		control_dependent_operation *p;

		assert(ctx->desc->kind == TARGET_VERTEX_SHADER);

		/* creating a store for the pilot variable */
		ESSL_CHECK(address = _essl_new_variable_reference_expression(ctx->pool, sym));
		_essl_ensure_compatible_node(address, rt_node->node);

		ESSL_CHECK(store = _essl_new_store_expression(ctx->pool, ADDRESS_SPACE_GLOBAL, address, out));
		_essl_ensure_compatible_node(store, rt_node->node);

		store->hdr.is_control_dependent = ESSL_TRUE;
		ESSL_CHECK(p = LIST_NEW(ctx->pool, control_dependent_operation));
		p->block = bb;
		p->op = store;
		p->next = NULL;
		ESSL_CHECK(_essl_ptrdict_insert(&pilot_func->control_flow_graph->control_dependence, store, p));
		LIST_INSERT_FRONT(&bb->control_dependent_ops, p);
		pilot_func->body = store;

	}
	/* we also need to fix (remove form structure) moved control dependent operations */
	ESSL_CHECK(fix_control_dependent_preds(ctx, rt_node->orig_func, pilot_func, copy));
	/* as we perform this moving several times we need to clear dictionary of copied operations
	 * because an operation can be needed for the next pilot shader */
	ESSL_CHECK(_essl_ptrdict_clear(&ctx->copied));

	return MEM_OK;

}

/**
 * analyzing whether a node is an initial candidate for the run time constant
 *
 * @param ctx optimization context
 * @param n node to analyze
 * @param block basic block where the node is located
 *
 * @returns MEM_OK if all allocations were correct
 */
static memerr collect_rt_nodes(pilot_calculations_context *ctx, node *n, basic_block *block)
{
	essl_bool is_const;

	ESSL_CHECK(_essl_ptrdict_clear(&ctx->rt_const_nodes));
	is_const = is_node_inputs_rt_constant(ctx, n, NULL, NULL);
	if(is_const)
	{
		node_succs_list *succs_list;
		run_time_constant_node *rtc_elem;
		essl_bool has_succs;

		succs_list = find_rt_const_succs(ctx, n, &has_succs);
		if(has_succs && succs_list == NULL) /* non null variant should be further optimized */
		{
			ESSL_CHECK(rtc_elem = LIST_NEW(ctx->temp_pool, run_time_constant_node));
			rtc_elem->node = n;
			rtc_elem->orig_rt_node = n;
			rtc_elem->orig_rt_bb = block;
			rtc_elem->orig_func = ctx->func;
			LIST_INSERT_FRONT(&ctx->rtc_nodes, rtc_elem);
		}
	}else if(0) /* disabled because MJOLL-2516 */
	{
		ptrdict_iter it;
		ptrdict_entry *entry;
		node *nn;
		_essl_ptrdict_iter_init(&it, &ctx->rt_const_nodes);
		while((entry = _essl_ptrdict_next_entry(&it)) != NULL)
		{

			node_succs_list *succs_list;
			run_time_constant_node *rtc_elem;
			essl_bool has_succs;
			nn = _essl_ptrdict_get_key(entry);
			if( nn == NULL)
			{
				break;
			}
			if(is_addressing_op(nn))
			{
				continue;
			}

			succs_list = find_rt_const_succs(ctx, nn, &has_succs);
			if(has_succs) /* non null variant should be further optimized */
			{
				ESSL_CHECK(rtc_elem = LIST_NEW(ctx->temp_pool, run_time_constant_node));
				rtc_elem->node = nn;
				rtc_elem->orig_rt_node = nn;
				rtc_elem->orig_rt_bb = block;
				rtc_elem->orig_func = ctx->func;
				LIST_INSERT_FRONT(&ctx->rtc_nodes, rtc_elem);
			}

		}

	}

	return MEM_OK;

}

/**
 * Finding  all initial candidates 
 */
static memerr find_constant_input_calculations_nodes(pilot_calculations_context *ctx)
{
	int j;

	for (j = ctx->cfg->n_blocks-1 ; j >= 0 ; j--)
	{
		basic_block *block = ctx->cfg->postorder_sequence[j];
		ESSL_CHECK(collect_successors(ctx, block));
	}

	for (j = ctx->cfg->n_blocks-1 ; j >= 0 ; j--)
	{
		basic_block *block = ctx->cfg->postorder_sequence[j];

		/* Scan control dependent ops */
		control_dependent_operation *cd;
		for (cd = block->control_dependent_ops ; cd != NULL ; cd = cd->next)
		{
			if(cd->op->hdr.kind == EXPR_KIND_STORE)
			{
				if(cd->op->expr.u.load_store.address_space != ADDRESS_SPACE_VERTEX_VARYING &&
					cd->op->expr.u.load_store.address_space != ADDRESS_SPACE_THREAD_LOCAL)
				{
					continue;
				}
			}
			ESSL_CHECK(collect_rt_nodes(ctx, cd->op, block));
		}

		if(block->source != NULL)
		{
			ESSL_CHECK(collect_rt_nodes(ctx, block->source, block));
		}
	}

	if(ctx->rtc_nodes != NULL)
	{
		find_last_point_for_hoisting_out(ctx);
	}

	return MEM_OK;
}

/**
 * Creating pilot shader function
 *
 * @param ctx optimization context
 * @param num number of the pilot shader
 *
 * @returns created pilot function
 */
static symbol * add_pilot_func(pilot_calculations_context *ctx, unsigned num)
{
	control_flow_graph *cfg;
	qualifier_set medp_qual;
	symbol_list *sl;
	symbol *pilot_func;
	const type_specifier *vec4;
	char func_name[29];

	assert(num < 5);
	snprintf(func_name, 29, "__gl_mali_pilot_shader_%d", num);
	_essl_init_qualifier_set(&medp_qual);
	medp_qual.precision = PREC_MEDIUM;

	ESSL_CHECK(vec4 = _essl_get_type_with_size(ctx->ts_ctx, TYPE_FLOAT, 4, SIZE_FP16));

	ESSL_CHECK(pilot_func = _essl_new_function_symbol(ctx->pool, _essl_cstring_to_string(ctx->pool, func_name), vec4, medp_qual, UNKNOWN_SOURCE_OFFSET));

	ESSL_CHECK(cfg = _essl_mempool_alloc(ctx->pool, sizeof(control_flow_graph)));

	ESSL_CHECK(_essl_ptrdict_init(&cfg->control_dependence, ctx->pool));
	pilot_func->control_flow_graph = cfg;

	ESSL_CHECK(cfg->entry_block = cfg->exit_block = _essl_new_basic_block(ctx->pool));

	cfg->exit_block->termination = TERM_KIND_EXIT;
	cfg->exit_block->n_successors = 0;
	cfg->exit_block->cost = 1.0;

	ESSL_CHECK(sl = LIST_NEW(ctx->pool, symbol_list));
	sl->sym = pilot_func;
	LIST_INSERT_BACK(&ctx->tu->functions, sl);

	ESSL_CHECK(sl = LIST_NEW(ctx->pool, symbol_list));
	sl->sym = pilot_func;
	LIST_INSERT_BACK(&ctx->tu->proactive_funcs, sl);
	pilot_func->opt.pilot.is_proactive_func = ESSL_TRUE;

	return pilot_func;

}

/**
 * Find most important run time constant calculation and move them to specail pilot shaders
 */
static memerr optimize_constant_input_calculations(pilot_calculations_context *ctx)
{
	if(ctx->rtc_nodes != NULL)
	{
		run_time_constant_node *elem;
		unsigned num = 0;
		
		/* we add at least 1 instructon and some additional overhead in the driver
		 * so the optimization will be applied if we remove at least 5 nodes from the main shader,
		 * but not more than 4*/
		for(elem = ctx->rtc_nodes; elem != NULL && num < 4; elem = elem->next)
		{
			if(elem->weight > MIN_ALLOWED_WEIGHT)
			{
				symbol *pilot_func;
				ESSL_CHECK(pilot_func = add_pilot_func(ctx, num));
				ESSL_CHECK(move_calculations_to_pilot_shader(ctx, pilot_func, elem, num));
				ctx->applied = ESSL_TRUE;
				num++;
			}
		}
	}
	return MEM_OK;
}

/**
 * Find run time constant calculations for the function
 */
static memerr find_constant_input_calculations_for_func(pilot_calculations_context *ctx, symbol *function)
{
	ctx->func = function;
	ctx->cfg = function->control_flow_graph;
	
	ESSL_CHECK(find_constant_input_calculations_nodes(ctx));

	return MEM_OK;
}

/**
 * Driver function for the optimization
 * To be called from middle passes run.
 */
memerr _essl_optimise_constant_input_calculations(pass_run_context *pr_ctx, translation_unit* tu)
{
	pilot_calculations_context ctx;
	symbol_list *sl;

	ctx.pool = pr_ctx->pool;
	ctx.temp_pool = pr_ctx->tmp_pool;
	ctx.desc = tu->desc;
	ctx.tu = tu;
	ctx.ts_ctx = pr_ctx->ts_ctx;
	ctx.rtc_nodes = NULL;
	ctx.applied = ESSL_FALSE;

	ESSL_CHECK(_essl_ptrdict_init(&ctx.node_succs, pr_ctx->tmp_pool));
	ESSL_CHECK(_essl_ptrset_init(&ctx.visited, pr_ctx->tmp_pool));
	ESSL_CHECK(_essl_ptrset_init(&ctx.hoist_points, pr_ctx->tmp_pool));
	ESSL_CHECK(_essl_ptrdict_init(&ctx.copied, pr_ctx->tmp_pool)) ;
	ESSL_CHECK(_essl_ptrdict_init(&ctx.rt_const_nodes, pr_ctx->tmp_pool));
	for(sl = tu->functions; sl != 0; sl = sl->next)
	{
		ESSL_CHECK(find_constant_input_calculations_for_func(&ctx, sl->sym));
	}

	ESSL_CHECK(optimize_constant_input_calculations(&ctx));
	if(ctx.applied)
	{
		for(sl = tu->functions; sl != 0; sl = sl->next)
		{
			ESSL_CHECK(_essl_compute_dominance_information(ctx.pool, sl->sym));
			_essl_validate_control_flow_graph(sl->sym->control_flow_graph);
		}
	}

	return MEM_OK;

}

