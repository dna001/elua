-- STM32F7DISCOVERY build configuration

return {
  cpu = 'stm32f769ni',
  components = {
    sercon = { uart = 0, speed = 115200 },
    romfs = true,
    advanced_shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 10, lua_lines = 50 },
    rpc = { uart = 1, speed = 115200 },
    xmodem = true,
    cints = true, 
    luaints = true
  },
  config = {
    egc = { mode = "alloc" },
    vtmr = { num = 4, freq = 10 },
    ram = { internal_rams = 1 },
    clocks = { external = 25000000, cpu = 216000000 },
    stm32f7_uart_pins = { con_rx_port = 0, con_rx_pin = 10, con_tx_port = 0, con_tx_pin = 9 }
  },
  modules = {
    generic = { 'all', "-i2c", "-net", "-spi", "-can", "-adc", "-pwm" },
    platform = 'all',
  },
}
