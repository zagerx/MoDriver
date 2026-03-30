#include "cia402.h"
void cia402_pds_not_ready_state(struct statemachine *sm)
{
	enum {
		CHECK_READY = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		/* 初始化状态：状态字 bit9=0, bit8=0, bit6=0, bit5=0, bit3=0, bit2=0, bit1=0, bit0=0
		 */
		if (inst && inst->statusword) {
			*inst->statusword &= ~0x014F; /* 清除相关位 */
		}
		sm->phase = CHECK_READY;
		break;

	case CHECK_READY:
		/* 检查是否完成初始化（如自检通过），然后自动切换到 Switch On Disabled */
		if (inst && inst->is_initialized) {
			// TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
		}
		break;

	case EXIT:
		/* 离开 Not Ready，进入 Switch On Disabled 时设置状态字 bit6=1, bit5=1 */
		if (inst && inst->statusword) {
			*inst->statusword = 0x0040; /* Switch On Disabled: bit6=1 */
		}
		break;

	default:
		break;
	}
}
