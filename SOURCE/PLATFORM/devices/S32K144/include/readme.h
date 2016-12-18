/**
** @ingroup soc_header
** @defgroup soc_header_usage Header File Usage
** @{
**  <p>
**  ## Introduction ##
**  
**  The following lines describe our recommendations of usage for the S32SDK
**  header files that are supposed to improve both code reuse and code
**  portability.\n
**  It presents typical use cases such as:
**      - 1. Initialize register
**      - 2. Initialize bit / bit-field
**      - 3. Set bit-field in register
**      - 4. Clear bit-field in register
**      - 5. Read bit / bit-field
**      - 6. Initialize bit using read/modify/write solution
**      - 7. Using bit-band to write individual bits
**      - 8. Using bit-band to read individual bits
**      - 9. Modifying register values when at least 1 bit is w1c
**      - 10. Using Interrupts
**      .
**  Any assignement of a hardcoded value is highly discouraged. It is recommended
**  the usage of variables or macros for code consistency and reuse reasons.\n
**  Information contained in the Header Files:
**      - Interrupt vector numbers\n
**        <b> Example: </b>
**        @code
**            HardFault_IRQn               = -13
**            MemoryManagement_IRQn        = -12
**            BusFault_IRQn                = -11
**            ...
**        @endcode
**
**      - Peripheral device memory map\n
**        <b> Example: </b>
**        @code
**            typedef struct {
**              ...
**              __IO uint32_t YOFS;
**              __IO uint32_t G;
**              __IO uint32_t UG;
**              __IO uint32_t CLPS;
**              ...
**              } ADC_Type, *ADC_MemMapPtr;
**        @endcode
**
**      - Peripheral device register access macros:\n
**          + a macro which speecifies the bit-field mask
**          + a macro which speecifies the bit-field offset
**          + a macro which speecifies the bit-field width
**          + a macro which maps a value to a bit-field
**          .
**        <b> Example: </b>
**        @code
**            #define ADC_YOFS_YOFS_MASK        0xFFu
**            #define ADC_YOFS_YOFS_SHIFT       0u
**            #define ADC_YOFS_YOFS_WIDTH       8u
**            #define ADC_YOFS_YOFS(x)          (((uint32_t)(((uint32_t)(x))<<ADC_YOFS_YOFS_SHIFT))&ADC_YOFS_YOFS_MASK)
**            ...
**        @endcode
**  
**  ## Usage examples: ##
**
**  ### 1. Initialize register \n###
**
**  This method is used to change the value of the entire register.\n
**  Should be used only when the previous value doesn't matter.\n
**
**  <b> General form: </b>
**  @code
**      <MODULE>_BASE_PTRS->regName = value;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      #define PIN_PTR        5U
**      PORTC->PCR[PIN_PTR] = regValue;
**  @endcode
**  
**  ### 2. Initialize bit / bit-field \n###
**
**  This method is used for initializing a bit or bit-field.\n
**  The benefit of using this method is that modifications of register addresses
**  or bit-field offsets will not require code changes when this method is used.
**  \n
**  <b> General form: </b>
**  @code
**      <MODULE>_BASE_PTRS->regName &= ~MASK;
**      <MODULE>_BASE_PTRS->regName |= (value << SHIFT) & MASK;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      #define PIN_IDX    5U
**      GPIO_PORT->PTOR &= ~GPIO_PTOR_PTTO_MASK;
**      GPIO_PORT->PTOR |= GPIO_PTOR_PTTO(PIN_IDX);
**  @endcode
**
**  ### 3. Set bit-field in register \n###
**
**  This method is used to perform a bitwise OR between a bit-field and a given
**  value.\n
**
**  <b> General form: </b>
**  @code
**      <MODULE>_BASE_PTRS->regName |= (value << shift) & MASK ;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      GPIO_PORT->PTOR |= (value << GPIO_PTOR_PTTO_SHIFT) & GPIO_PTOR_PTTO_MASK;
**  @endcode
**
**  ### 4. Clear bit-field in register \n###
**
**  This method is used for clearing a bit-field in a register.\n
**  For registers where there is at least one w1c bit please see section 8.\n
**
**  <b> General form: </b>
**  @code
**      <MODULE>_BASE_PTRS->regName &= ~MASK;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      GPIO_PORT->PTOR &= ~GPIO_PTOR_PTTO_MASK;
**  @endcode
**
**  ### 5. Read bit / bit-field \n###
**
**  This method is used for reading the value of a bit-field from a register.
**
**  <b> General form: </b>
**  @code
**      x = (<MODULE>_BASE_PTRS->regName & mask) >> shift
**  @endcode
**
**  <b> Example: </b>
**  @code
**      pcr_mux_value = (base->PCR[pin] & PORT_PCR_MUX_MASK) >> PORT_PCR_MUX_SHIFT;
**  @endcode
**  
**  ### 6. Initialize bit using read/modify/write solution ###
**
**  This method is used for clearing / setting a value to a bit in a
**  register taking into consideration the previous value.\n
**  For w1c bits please consult section 8.
**
**  <b> General form: </b>
**  @code
**      regValue                     = <MODULE>_BASE_PTRS->regName;
**      regValue                    &= ~MASK;
**      regValue                    |= (value << shift) & MASK;
**      <MODULE>_BASE_PTRS->regName  = regValue;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      regValue        = base->PCR[pin];
**      regValue       &= ~(PORT_PCR_MUX_MASK);
**      regValue       |= PORT_PCR_MUX(pcr_mux_value);
**      base->PCR[pin]  = regValue;
**  @endcode
**  
**  ### 7. Using bit-band to write individual bits \n###
**
**  This method is used to write a value to a bit in a register using bit-band
**  access.\n
**  Writing a bit that is w1c using bit-band access might result in the
**  clearing of that bit. For a solution to this problem please consult
**  section 8
**
**  <b> General form: </b>
**  @code
**      BITBAND_ACCESS32(reg_address, shift) = value;
**  @endcode
**
**  <b> Example: </b>
**  @code
**      BITBAND_ACCESS32(&(base->PCR[pin]), PORT_PCR_PS_SHIFT) = 1;
**  @endcode
**  
**  ### 8. Using bit-band to read individual bits \n###
**
**  This method is used to read a value of a bit from a register using bit-band
**  access.\n
**
**  <b> General form: </b>
**  @code
**      x = BITBAND_ACCESS32(reg_address, shift);
**  @endcode
**
**  <b> Example: </b>
**  @code
**      x = BITBAND_ACCESS32(&(base->PCR[pin]), PORT_PCR_PS_SHIFT);
**  @endcode
**
**  ### 9. Modifying register values when at least 1 bit is w1c \n###
**  When a register has at least one W1C bit different approaches are suggested
**  depending on each particular case:
**      - clearing a w1c bit in registers where there are only w1c bits\n
**          For this case it is recommended that the bit mask is written at the
**          register address.\n
**          <b> General form: </b>
**          @code
**              <MODULE>_BASE_PTRS->regName = MASK;
**          @endcode
**      
**          <b> Example: </b>
**          @code 
**              LPI2C_BASE_PTRS->MSR = LPI2C_MSR_RDF_MASK;
**          @endcode
**
**      - clearing a w1c bit in registers where there are also "normal" bits\n
**          For this case it is recommended that a read-modify-write method is
**          used with the mask of the bit-field that will leave the values of
**          the other bits unchanged.\n
**          <b> General form: </b>
**          @code
**              <MODULE>_BASE_PTRS->regName |= MASK;
**          @endcode
**      
**          <b> Example: </b>
**          @code
**              CMP_BASE_PTRS->C0 |= CMP_C0_CFF_MASK;
**          @endcode
**
**      - clearing a "normal" bit-field in registers where there are also w1c 
**        bits \n
**          For this case it is recommended that a special mask is applied to
**          the register. The mask should be created by applying an AND
**          operation between the negated mask of the bit-field that is
**          intended to be cleared and the negated masks of all the w1c 
**          bit-fields.\n
**          <b> General form: </b>
**          @code
**              MASK = ~BITFIELD_MASK & ~W1C_BITFIELDS_MASKS
**              <MODULE>_BASE_PTRS->regName &= MASK;
**          @endcode
**      
**          <b> Example: </b>
**          @code
**              MASK = ~CMP_C0_SE_MASK & (~CMP_C0_CFR_MASK & ~CMP_C0_CFF_MASK);
**              CMP_BASE_PTRS->C0 &= MASK;
**          @endcode
**
**      - setting a "normal" bit-field in registers where there are also w1c 
**        bits \n
**          For this case it is recommended that a clearing is done as in the
**          previous step and then the bit-field is set as in step 3\n
**          <b> General form: </b>
**          @code
**              MASK = ~BITFIELD_MASK & ~W1C_BITFIELDS_MASKS
**              <MODULE>_BASE_PTRS->regName &= MASK;
**              <MODULE>_BASE_PTRS->regName |= (value << shift) & MASK;
**          @endcode
**      
**          <b> Example: </b>
**          @code
**              MASK = ~CMP_C0_SE_MASK & (~CMP_C0_CFR_MASK & ~CMP_C0_CFF_MASK);
**              CMP_BASE_PTRS->C0 &= MASK;
**              CMP_BASE_PTRS->C0 |= (value<<CMP_C0_SE_SHIFT) & CMP_C0_SE_MASK;
**          @endcode
**
**  ### 10. Using Interrupts ###
**
**  <b> Example: </b>\n
**  Enable WakeUp interrupt for instance = 0
**  @code
**      uint32_t instance = 0;
**
**      const IRQn_Type g_flexcanWakeUpIrqId[] = CAN_Wake_Up_IRQS;
**
**      INT_SYS_EnableIRQ(g_flexcanWakeUpIrqId[instance]);
**  @endcode
**  </p>
**  @}
*/