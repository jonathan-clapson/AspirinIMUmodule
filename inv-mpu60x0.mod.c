#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x5b800c16, "module_layout" },
	{ 0xd374745e, "iio_triggered_buffer_cleanup" },
	{ 0x74618eee, "iio_trigger_unregister" },
	{ 0xf9a482f9, "msleep" },
	{ 0x13d0adf7, "__kfifo_out" },
	{ 0xdf81aa63, "bus_find_device_by_name" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0xded97268, "dev_set_drvdata" },
	{ 0x85833915, "iio_read_const_attr" },
	{ 0x2ceed69c, "iio_trigger_notify_done" },
	{ 0x1e8c15fd, "iio_trigger_alloc" },
	{ 0x46608fa0, "getnstimeofday" },
	{ 0xbed60566, "sub_preempt_count" },
	{ 0x62b72b0d, "mutex_unlock" },
	{ 0x694ab8fe, "iio_device_register" },
	{ 0x91715312, "sprintf" },
	{ 0x93d13daa, "iio_device_unregister" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x150b3e5a, "spi_busnum_to_master" },
	{ 0x6977306a, "dev_err" },
	{ 0x27e1a049, "printk" },
	{ 0xb9d8941c, "driver_unregister" },
	{ 0x73e20c1c, "strlcpy" },
	{ 0xe16b893b, "mutex_lock" },
	{ 0x4c6ff041, "add_preempt_count" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x373db350, "kstrtoint" },
	{ 0x43b0c9c3, "preempt_schedule" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0x151fe218, "spi_sync" },
	{ 0x276d1deb, "iio_device_free" },
	{ 0xbadf804b, "put_device" },
	{ 0x9bc67d74, "spi_alloc_device" },
	{ 0xf6e5184a, "iio_trigger_register" },
	{ 0x7985e538, "iio_device_alloc" },
	{ 0x48f9a177, "iio_trigger_free" },
	{ 0x4a191e83, "spi_add_device" },
	{ 0x2d6bcdcb, "iio_trigger_generic_data_rdy_poll" },
	{ 0x954ae616, "spi_get_device_id" },
	{ 0xb9d919a0, "iio_push_to_buffers" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0xf23fcb99, "__kfifo_in" },
	{ 0xb81960ca, "snprintf" },
	{ 0xc0d4826c, "spi_register_driver" },
	{ 0xa71f8b83, "dev_get_drvdata" },
	{ 0x34952774, "iio_triggered_buffer_setup" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=industrialio-triggered-buffer,industrialio";

MODULE_ALIAS("spi:mpu60x0");

MODULE_INFO(srcversion, "1D11198EA824BD327EC9C9F");
