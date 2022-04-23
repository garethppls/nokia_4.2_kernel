#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define jtag_id_readl(drvdata, off)		__raw_readl(drvdata->base + off)

#define JTAG_ID		(0x000)

static uint32_t jtag_id = 0;

struct jtag_id_drvdata {
	void __iomem		*base;
	struct device		*dev;
};

static struct jtag_id_drvdata *jtagiddrvdata;

static int jtag_id_read(struct seq_file *m, void *v)
{
	struct jtag_id_drvdata *drvdata = jtagiddrvdata;

	if (!drvdata)
		return false;

	if (jtag_id == 0)
		jtag_id = jtag_id_readl(drvdata, JTAG_ID);

	dev_dbg(drvdata->dev, "jtag id register: %x\n", jtag_id);

	seq_printf(m, "0x%x\n", jtag_id);

	return 0;
}

static int jtag_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, jtag_id_read, NULL);
}

static const struct file_operations jtag_id_fops = {
	.open		= jtag_id_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void jtag_id_create_proc(void)
{
	struct proc_dir_entry *entry;
	entry = proc_create("jtag_id", 0 /* default mode */,
			NULL /* parent dir */, &jtag_id_fops);
}

static int jtag_id_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jtag_id_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	/* Store the driver data pointer for use in exported functions */
	jtagiddrvdata = drvdata;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "jtag-id-base");
	if (!res)
		return -ENODEV;

	drvdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->base)
		return -ENOMEM;

	jtag_id_create_proc();
	dev_info(dev, "jtag-id interface initialized\n");
	return 0;
}

static int jtag_id_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id jtag_id_match[] = {
	{.compatible = "qcom,jtag-id"},
	{}
};

static struct platform_driver jtag_id_driver = {
	.probe          = jtag_id_probe,
	.remove         = jtag_id_remove,
	.driver         = {
		.name   = "msm-jtag-id",
		.owner	= THIS_MODULE,
		.of_match_table = jtag_id_match,
	},
};

static int __init jtag_id_init(void)
{
	return platform_driver_register(&jtag_id_driver);
}
arch_initcall(jtag_id_init);

static void __exit jtag_id_exit(void)
{
	platform_driver_unregister(&jtag_id_driver);
}
module_exit(jtag_id_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("JTag ID driver");
