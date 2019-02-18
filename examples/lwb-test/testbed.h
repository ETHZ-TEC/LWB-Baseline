#ifndef TESTBED_H_
#define TESTBED_H_

#define TB_NUMPATTERN 8
#define TB_NUMNODES 8

// GPIO pin used to signal the availability of data in the EEPROM
#define EVENT_PIN 6

typedef struct 
{
	uint8_t traffic_pattern;             // 0:unused, 1:p2p, 2:p2mp, 3:mp2p, 4: mp2mp
	uint8_t source_id[TB_NUMNODES];      // Only source_id[0] is used for p2p/p2mp 
	uint8_t destination_id[TB_NUMNODES]; // Only destination_id[0] is used for p2p/mp2p
	uint8_t msg_length;                  // Message length in bytes in/to EEPROM
	uint8_t msg_offsetH;                 // Message offset in bytes in EEPROM (high byte)
	uint8_t msg_offsetL;                 // Message offset in bytes in EEPROM (low byte)

	uint32_t periodicity;                // Period in ms (0 indicates aperiodic traffic)
	uint32_t aperiodic_upper_bound;      // Upper bound for aperiodic traffic in ms
	uint32_t aperiodic_lower_bound;      // Lower bound for aperiodic traffic in ms
} pattern_t;

typedef struct
{
	pattern_t p[TB_NUMPATTERN];          // Up to TB_NUMPATTERN parallel configurations
} config_t;


// Helper functions to print the input parameters injected by the competition's testbed
void
print_testbed_pattern(pattern_t* p)
{
	uint8_t i;
	printf("    Traffic pattern: ");
	switch(p->traffic_pattern)
	{
		case 0: printf("unused\n");
			break;
		case 1: printf("P2P\n");
			break;
		case 2: printf("P2MP\n");
			break;
		case 3: printf("MP2P\n");
			break;
		case 4: printf("MP2MP\n");
			break;
		default: printf("Unknown\n");
	}
	if( (p->traffic_pattern>0) && (p->traffic_pattern <=4))
	{
		printf("    Sources:\n");
		for(i=0;i<TB_NUMNODES;i++)
		{
			if(p->source_id[i]!=0)
			printf("      %d: %d\n",i,p->source_id[i]);
		}
		printf("    Destinations:\n");
		for(i=0;i<TB_NUMNODES;i++)
		{
			if(p->destination_id[i]!=0)
			printf("      %d: %d\n",i,p->destination_id[i]);
		}
		if(p->periodicity==0)
		{
			printf("    Aperiodic Upper: %lu\n",p->aperiodic_upper_bound);
			printf("    Aperiodic Lower: %lu\n",p->aperiodic_lower_bound);
		}
		else
		{
			printf("    Period: %lu\n",p->periodicity);
		}

		printf("    Message Length: %d\n",p->msg_length);
		printf("    Message OffsetH: %d\n",p->msg_offsetH);
		printf("    Message OffsetL: %d\n",p->msg_offsetL);
	}
	printf("\n");

}

void
print_testbed_config(config_t* cfg)
{
	printf("Testbed configuration:\n");
	uint8_t i;
	for(i=0;i<TB_NUMPATTERN;i++)
	{
	        printf("  Pattern %d:\n",i);
		print_testbed_pattern(&(cfg->p[i]));
	}
}

#endif

