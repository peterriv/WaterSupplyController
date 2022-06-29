
#include "hex2ascii.h"
#include "stdint.h"

// Added bytes (digits) order swapping in buffer
// VERSION		11

// Преобразование чисел из HEX в DEC  и в ASCII, т.е. в ASCII будут десятичные
// цифры исходного шестнадц. числа, и запись в передаваемый буфер, начиная с
// младшего разряда к старшему. Размер буфера - не менее 5-ти байт
// Пример: 0xFFFE=65534 -> buf[0-4]={0x34,0x33,0x35,0x35,0x36}
void Hex2Dec2ASCII(uint16_t hex_num, uint8_t * buf, uint16_t buf_size)
{
	uint8_t most_sig_digit=0,digit4=0,digit3=0,digit2=0,least_sig_digit=0;
	
	if (buf_size<5) return;
	
  while (hex_num>=10000)
  {
   hex_num-=10000;
   most_sig_digit++;
  }
  while (hex_num>=1000)
  {
   hex_num-=1000;
   digit4++;
  }
  while (hex_num>=100)
  {
   hex_num-=100;
   digit3++;
  }
  while (hex_num>=10)
  {
   hex_num-=10;
   digit2++;
  }
  while (hex_num>=1)
  {
   hex_num-=1;
   least_sig_digit++;
  }

  // Приводим к ASCII    
  most_sig_digit+=0x30;
  digit4+=0x30;
  digit3+=0x30;
  digit2+=0x30;
  least_sig_digit+=0x30;
	
  // Проверяем на 0 старшие(левые разряды), если да, то заменяем на пробелы
	/*if (most_sig_digit==0x30)
	{
		most_sig_digit=0x20;
		
		if (digit4==0x30)
		{
			digit4=0x20;
			
			if (digit3==0x30)
			{
				digit3=0x20;
				
				if (digit2==0x30)
				{
					digit2=0x20;
				}
			}
		}
	}*/
 
	buf[4]=most_sig_digit;
  buf[3]=digit4;
  buf[2]=digit3;
  buf[1]=digit2;
  buf[0]=least_sig_digit;
}


// Преобразование чисел из ASCII в DEC и в HEX, т.е. в ASCII записаны десятичные
// цифры исходного числа. В п.п. передаётся адрес буфера с ASCII символами чисел,
// начиная с младшего разряда числа. Размер буфера с ASCII символами >0 и <=5 байт
// Пример: buf[0-4]={0x34,0x33,0x35,0x35,0x36} -> 65534=0xFFFE
// Пример: buf[0-1]={0x35,0x36} -> 65=0x0041
// Пример: buf[0]={0x36} -> 6=0x0006
uint16_t ASCII2Dec2Hex( uint8_t * buf, uint16_t buf_size)
{
	uint8_t most_sig_digit = 0, digit4 = 0, digit3 = 0, digit2 = 0, least_sig_digit;
	uint16_t sigma;
	
	if (buf_size==0) return 0;
	if (buf_size>5) return 0;

	// Хоть один символ будет всегда, он же самый младший разряд числа
	least_sig_digit=buf[0];
	// Приводим к обычным числам, вместо символов ASCII    
	least_sig_digit-=48;
	
	
	// Если присутствует 2-й по старшинству символ
	if (buf_size>1)
	{
		digit2=buf[1];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit2==0x20) digit2=0x30;
		
		// Приводим к обычным числам, вместо символов ASCII    
		digit2-=48;
	}
	
	// Если присутствует 3-й по старшинству символ
	if (buf_size>2)
	{
		digit3=buf[2];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit3==0x20) digit3=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit3-=48;
	}

	// Если присутствует 4-й по старшинству символ
	if (buf_size>3)
	{
		digit4=buf[3];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit4==0x20) digit4=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit4-=48;
	}
	
	// Если присутствует 5-й по старшинству символ
	if (buf_size>4)
	{
		most_sig_digit=buf[4];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (most_sig_digit==0x20) most_sig_digit=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		most_sig_digit-=48;
	}

	// Преобразование из DEC в HEX
	sigma=0;
	if (buf_size>4)
	{
		while (most_sig_digit>=1)
		{
			most_sig_digit--;
			sigma+=0x2710;
		}
	}

	if (buf_size>3)
	{
		while (digit4>=1)
		{
			digit4--;
			sigma+=0x03E8;
		}
	}

	if (buf_size>2)
	{
		while (digit3>=1)
		{
			digit3--;
			sigma+=0x0064;
		}
	}

	if (buf_size>1)
	{
		while (digit2>=1)
		{
			digit2--;
			sigma+=0x000A;
		}
	}

	while (least_sig_digit>=1)
	{
		least_sig_digit--;
		sigma+=0x0001;
	}

	return sigma;
}


// Преобразование чисел из ASCII в DEC и в HEX, т.е. в ASCII записаны десятичные
// цифры исходного числа. В п.п. передаётся адрес буфера с ASCII символами чисел,
// начиная с младшего разряда числа. Размер буфера с ASCII символами >0 и <=9 байт
// Пример: buf[0-6]={0x39,0x32,0x34,0x33,0x35,0x35,0x36} -> 6553429=0x63FF55
// Пример: buf[0-1]={0x35,0x36} -> 65=0x0041
// Пример: buf[0]={0x36} -> 6=0x0006
uint32_t ASCII2Dec2Hex_u32( uint8_t * buf, uint16_t buf_size)
{
	uint8_t most_sig_digit = 0, digit8 = 0, digit7 = 0, digit6 = 0, digit5 = 0;
	uint8_t digit4 = 0 , digit3 = 0, digit2 = 0, least_sig_digit;
	uint32_t sigma;
	
	if (buf_size==0) return 0;
	if (buf_size>9) return 0;

	// Хоть один символ будет всегда, он же самый младший разряд числа
	least_sig_digit=buf[0];
	// Приводим к обычным числам, вместо символов ASCII    
	least_sig_digit-=48;
	
	
	// Если присутствует 2-й по старшинству символ
	if (buf_size>1)
	{
		digit2=buf[1];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit2==0x20) digit2=0x30;
		
		// Приводим к обычным числам, вместо символов ASCII    
		digit2-=48;
	}
	
	// Если присутствует 3-й по старшинству символ
	if (buf_size>2)
	{
		digit3=buf[2];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit3==0x20) digit3=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit3-=48;
	}

	// Если присутствует 4-й по старшинству символ
	if (buf_size>3)
	{
		digit4=buf[3];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit4==0x20) digit4=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit4-=48;
	}

	// Если присутствует 5-й по старшинству символ
	if (buf_size>4)
	{
		digit5=buf[4];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit5==0x20) digit5=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit5-=48;
	}

	// Если присутствует 6-й по старшинству символ
	if (buf_size>5)
	{
		digit6=buf[5];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit6==0x20) digit6=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit6-=48;
	}

	// Если присутствует 7-й по старшинству символ
	if (buf_size>6)
	{
		digit7=buf[6];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit7==0x20) digit7=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit7-=48;
	}

	// Если присутствует 8-й по старшинству символ
	if (buf_size>7)
	{
		digit8=buf[7];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (digit8==0x20) digit8=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		digit8-=48;
	}
	
	// Если присутствует 9-й по старшинству символ
	if (buf_size>8)
	{
		most_sig_digit=buf[8];

		// Проверяем на код пробела и заменяем на ASCII код нуля
		if (most_sig_digit==0x20) most_sig_digit=0x30;

		// Приводим к обычным числам, вместо символов ASCII    
		most_sig_digit-=48;
	}

	// Преобразование из DEC в HEX
	sigma=0;

	if (buf_size>8)
	{
		while (most_sig_digit>=1)
		{
			most_sig_digit--;
			sigma+=0x05F5E100;
		}
	}

	if (buf_size>7)
	{
		while (digit8>=1)
		{
			digit8--;
			sigma+=0x989680;
		}
	}

	if (buf_size>6)
	{
		while (digit7>=1)
		{
			digit7--;
			sigma+=0x0F4240;
		}
	}

	if (buf_size>5)
	{
		while (digit6>=1)
		{
			digit6--;
			sigma+=0x0186A0;
		}
	}
	
	if (buf_size>4)
	{
		while (digit5>=1)
		{
			digit5--;
			sigma+=0x2710;
		}
	}

	if (buf_size>3)
	{
		while (digit4>=1)
		{
			digit4--;
			sigma+=0x03E8;
		}
	}

	if (buf_size>2)
	{
		while (digit3>=1)
		{
			digit3--;
			sigma+=0x0064;
		}
	}

	if (buf_size>1)
	{
		while (digit2>=1)
		{
			digit2--;
			sigma+=0x000A;
		}
	}

	while (least_sig_digit>=1)
	{
		least_sig_digit--;
		sigma+=0x0001;
	}

	return sigma;
}


// Преобразование чисел из HEX в ASCII, т.е. в ASCII будут символы шестнадцатиричных
// цифр исходного шестнадц. числа. Запись в передаваемый буфер, начиная с
// младшего разряда числа. Размер буфера - не менее 2-х байт
// Пример: 0xFE -> buf[0-1]={0x45,0x46}
void Hex2ASCII(uint8_t hex_num, uint8_t * buf, uint16_t buf_size)
{
	uint8_t msb,lsb;
	
	if (buf_size<2) return;
	
	msb=hex_num;
	lsb=hex_num;
	
	msb&=0xF0;
	msb>>=4;
	lsb&=0x0F;

	// Перевод старшей тетрады
	if (msb>=10)
	{
		msb+=55;
	}
	else
	{
		msb+=0x30;
	}

	// Перевод младшей тетрады
	if (lsb>=10)
	{
		lsb+=55;
	}
	else
	{
		lsb+=0x30;
	}    

  buf[0]=lsb;
  buf[1]=msb;
}


// Преобразование чисел из ASCII в HEX, т.е. в ASCII записаны символы шестнадцатиричных
// цифр исходного шестнадц. числа. Запись в передаваемый буфер, начиная с
// младшего разряда числа. Размер буфера - не менее 2-х байт
// Пример:  buf[0-1]={0x45,0x46} -> 0xFE
uint8_t ASCII2HEX (uint8_t msbtemp, uint8_t lsbtemp)
{
	uint8_t temp;
	
	if (msbtemp>=65) msbtemp-=55;   
	else msbtemp-=0x30;
	// Перевод младшей тетрады
	if (lsbtemp>=65) lsbtemp-=55;
	else lsbtemp-=0x30;   
	temp=msbtemp;
	temp<<=4;
	temp+=lsbtemp;
	return temp;
}


// Функция преобразования строки в число
// Строка чисел длиной до 20 символов включительно
// Первый символ в строке соответствует старшему разряду числа
int64_t String2number(const char* str, uint8_t length)
{
	uint8_t sign = 0;
	int64_t result = 0;

	if(length > 20) return 0;

	sign = (*str == '-');
	
	if (sign)
	{
		++str;
		length--;
	}
	while (length){
			if ((*str >= '0') && (*str <= '9'))
			{
				result += (*str++ & 0x0F);
				if (--length != 0) result *= 10;
			}
		}
	if (sign) return -result;
	else      return  result;
}


// Создаёт строковой формат числа в куче и возвращает указатель на строку в стиле С
//const char* numbercsz(signed long long  number) {
//    char* result = new char[21], *index = result;
//    if (number < 0) {
//        *index++ = '-';
//        number = -number;
//    }
//   
//    while (number > 0) {
//        *index++ = (number % 10) | '0';
//        number /= 10;
//    }
//    char* end = result + 21;
//    while (index != end) *index++ = '\0';
//    return(result);
//}


// Swapping order of bytes in buffer ( last with first, last - 1 with first + 1, etc)
void Buffer_bytes_swap(uint8_t * buf, uint32_t buf_size)
{
	uint8_t temp1_u8, temp2_u8;
	uint32_t idx1, idx2;
	
	idx2 = buf_size - 1;
	for(idx1 = 0; idx1 < buf_size; idx1++)
	{
		temp1_u8 = buf[idx1];
		buf[idx1] = buf[idx2];
		buf[idx2--] = temp1_u8;
		
		if(idx1 >= idx2) break;
	}
}

