#
# Makefile for miscellaneous I2C chip drivers.
#
obj-$(CONFIG_SENSORS_DS1337)	+= ds1337.o
obj-$(CONFIG_SENSORS_DS1374)	+= ds1374.o
obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_SENSORS_EEPROM)	+= eeprom.o
obj-$(CONFIG_SENSORS_MAX6875)	+= max6875.o
obj-$(CONFIG_SENSORS_M41T00)	+= m41t00.o
obj-$(CONFIG_SENSORS_PCA9539)	+= pca9539.o
obj-$(CONFIG_SENSORS_PCA963X)	+= pca963x.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_TPS65010)		+= tps65010.o
obj-$(CONFIG_SENSORS_TLV320AIC23) += tlv320aic23.o
obj-$(CONFIG_GPIOEXPANDER_OMAP)	+= gpio_expander_omap.o
obj-$(CONFIG_MENELAUS)		+= menelaus.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_TWL4030_CORE)	+= twl4030_core.o
obj-$(CONFIG_TWL4030_CORE)	+= twl4030_power.o
obj-$(CONFIG_TWL4030_GPIO)	+= twl4030_gpio.o
obj-$(CONFIG_TWL4030_USB)	+= twl4030_usb.o charger_detector.o
obj-$(CONFIG_TWL4030_MADC)	+= twl4030_madc.o
obj-$(CONFIG_TWL4030_POWEROFF)	+= twl4030_poweroff.o
obj-$(CONFIG_RTC_X1205_I2C)	+= x1205.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

