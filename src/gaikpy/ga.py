
#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

Genetic Algoritm module 

author: Erik Strahl

Year: 2020

"""

import numpy as np
import random
import math

import logging
import sys
logger = logging.getLogger("gaikpy_logger")
module=sys.modules[__name__]
logger.info("Logging started on  " + str(module))

class ga_world(object):
    """ General Genetic Algorithm Object (World)

    """

    def __init__(self,chrom_length,bounds,pop_size=8,num_elites=3,num_generations=10000,mut_rate=0.36,
                    eval_func=None,local_optimiser=None,accuracy_reached=None):
        """Constructor of Genetic Algorithm World 

        Parameters
        ----------
        chrom_length : int
            Number of Chromosomes
        bounds : array
            boundyries of every single chromosome
        pop_size : int, optional
            Number of the worlds population, by default 8
        num_elites : int, optional
            Number of elite individuals, by default 3
        num_generations : int, optional
            Number of the generations, by default 10000
        mut_rate : float, optional
            Mutation rate, by default 0.36
        eval_func : python function, optional
            Evaluation function; external function that calculates the fitness 
                of the individual (fitness function), by default None
        local_optimiser : python function, optional
            Local optimizer function; Perfomace of the GA can sometimes improved by improving 
            local minima with another another algorithm, by default None
        accuracy_reached : python function, optional
            external function to return a boolean, if the needed accuracy is reached;
            this is depending strongly on the application domain;
             by default None
        """

        self.chrom_length=chrom_length
        self.bounds=bounds
        self.pop_size=pop_size
        self.num_elites=num_elites
        self.num_generations=num_generations
        self.mut_rate=mut_rate
        self.eval_func=eval_func
        self.local_optimiser=local_optimiser
        self.accuracy_reached=accuracy_reached
        

    class individual(object):
        """Object for one individual of the population

        """
        def __init__(self, gene ,eval_func):
            """[summary]

            Parameters
            ----------
            gene : array
                initialization value of the gene
            eval_func : python function
                fitness function 

            Raises
            ------
            NameError
                Raises error if gene is None
            """

            if gene is None:
                raise NameError('None not allowed in __init__!')
            self._gene = gene
            self.eval_func=eval_func            
            logger.debug (" gen init: " + str(self._gene))
            self.fitness = eval_func(self._gene)
            

        def change_gene(self, index, value):
            self._gene[index] = value
            #self.fitness = eval_func(self.gene)
        
        def set_chromo(self, chromo):

            if chromo is None:
                raise NameError('None not allowed in set_chromo!')

            self._gene = chromo
            #self.fitness = eval_func(self.gene)

        def get_chromo(self):
            return(self._gene) 
            #self.fitness = eval_func(self.gene)

        def refresh_fitness(self):
            
            #Calculate the firness with eval function
            logger.debug (" refresh fitness: " + str(self._gene))
            self.fitness=self.eval_func(self._gene)

        def __repr__(self):
            return '{}: {} f: {}'.format(self.__class__.__name__,
                                            self._gene,
                                            self.fitness)

    class Population(object):
        """ Object for the whole population of a world

        """
        def __init__(self):
            self.individuals = []
            self.sorted = False

    @staticmethod
    def tournamentSelection(pop, num):
        """ Runs num rounds of a tournament selection process on the given population
        and return the best fittest individual of all the rounds

        Parameters
        ----------
        pop : Population object
            Population to run the tournament with
        num : int
            Number of rounds in the tournament

        Returns
        -------
        [type]
            [description]
        """

        #Get a randdom individua out of the population as the current leader
        indi_num = random.randint(0, len(pop.individuals) - 1)

        #run the rounds
        for i in range(num):

            # take a random individual of the population     
            indi_other = random.randint(0, len(pop.individuals) - 1)
            if pop.individuals[indi_other].fitness < pop.individuals[indi_num].fitness:
                # if the other individual is better than the recent leader take it as new leader 
                indi_num = indi_other
        return indi_num

    @staticmethod
    def gen_crossover(g_mom, g_dad):
        """ Genetic Crossover function- Take two individuals and mix their genes with a random weight.

        Parameters
        ----------
        g_mom : Array 
            Genes of one individual 
            
        g_dad : Array
            Genes of the other individual

        Returns
        -------
        Array
            The crossovered genes
        """
        off = []
        weight = random.random()
        for i in range(len(g_mom)):
            off.append((g_mom[i] * weight + g_dad[i] * (1 - weight)))
            #            if random.random() > 0.5:
            #                brother[i] = ((gMom[i] + gDad[i]) / 2) + (gMom[i] - gDad[i]) - random.random()
            #                # brother=gDad.clone()
            #            else:
            #                brother[i] = ((gMom[i] + gDad[i]) / 2) + (gDad[i] - gMom[i]) + random.random()
            # brother=gMom.clone()

        return (off)

    
    def mutate(self,indi):
        """ Mutate an individual

        Parameters
        ----------
        indi : Individual object
            Individual that should be mutated

        Returns
        -------
        array
            Changed gene set
        """
        off = []
        for i in range(len(indi.get_chromo())):
            off.append(indi.get_chromo()[i])
            # if random.random()<indi.fitness*5:
            if True:
                if random.random() < self.mut_rate:
                    # !!!! ToDo Strength of mutation, dependend on success
                    off[i] = off[i] + (random.random() * 2 - 1)

        indi.set_chromo(off)
        logger.debug ( "Mutation : gene: " + str(off))
        # raw_input()
        return (off)

    @staticmethod
    def init_gene(chrome_length,bounds=None):
        """Initialise a gene

        Parameters
        ----------
        chrome_length : int
            length of the chromosome
        bounds : array, optional
            bounds of the gene, by default None
        """

        logger.debug ("init gene")
        logger.debug("len bounds: " + str(len(bounds)))
        logger.debug("bounds: " + str(bounds))
        logger.debug("chrome_length: " + str(chrome_length))
        import math
        chrome = np.zeros(chrome_length)
        for t in range(chrome_length):
            if (bounds == None):
                init = random.uniform(-math.pi, math.pi)
            else:
                up, down = bounds[t]
                init = random.uniform(up, down)
            chrome[t] = init
        # Take out or Leave in: Optimize at creation
        #chrome=chain.active_from_full(chain.inverse_kinematics(target_frame, initial target_=chain.active_to_full(chrome, starting_nodes_angles), method="SLSQP",include_orientation=True))
        return(chrome)

    def run(self):
        """ run the world
        """

        # initialize genomes
        pop = self.Population()
        for i in range(self.pop_size):
            chrome = self.init_gene(self.chrom_length,bounds=self.bounds)
            #chrome = np.zeros(chromLength)
            # for t in range(chromLength):
            #    init=random.uniform(-3.141, 3.141)
            #    chrome[t]=init
            # !!!! ToDo change parameter of SLSQP optimization. Do not go so deep to save calculation time
            #chrome=chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(chrome, starting_nodes_angles), method="SLSQP",include_orientation=True, max_iter=max_iter))
            # indi=individual(chrome)
            indi = self.individual(self.init_gene(self.chrom_length,self.bounds),self.eval_func)
            pop.individuals.append(indi)
        #chrome = chain.active_from_full(chain.inverse_kinematics(target_frame, method="SLSQP",include_orientation=True, max_iter=max_iter))
        # indi=individual(chrome)
        indi = self.individual(self.init_gene(self.chrom_length,self.bounds),self.eval_func)
        pop.individuals.append(indi)

        pop.individuals = sorted(
            pop.individuals, key=lambda individual: individual.fitness)

        i = 0
        nic = 0  # No improvement counter
        minFitness = 10
        lastFitness = 10
        min_distance = 10
        min_orientation_distance = 10
        acc_reached = False

        # while i < numGenerations and minFitness > reachFitness:  # iterate through the generations
        while i < self.num_generations and not acc_reached:  # iterate through the generations
            #        if lastFitness-minFitness<0.01:
            #		nic+=1
            #        else:
            #			lastFitness=minFitness
            #			nic=0

            i += 1
            logger.debug("(Gen: #%s) Total error: %s\n" % (i, np.sum([ind.fitness for ind in pop.individuals])))
            logger.debug("Min Error: " + str(pop.individuals[0].fitness))
            new_pop = self.Population()
            # winners = np.zeros((params[4], params[3])) #20x2
            # clear population from individuals with the same fitness
            t = 1

            # Get all individiduals out, that are nearly equal
            while (t < len(pop.individuals)):

                if np.allclose(pop.individuals[t-1].fitness, pop.individuals[t].fitness, rtol=1e-03, atol=1e-04):
                    pop.individuals[t].gene = self.init_gene(self.chrom_length,bounds=self.bounds)
                    logger.debug ("cleared an individual cause too equal")
                    
                t += 1


            # get the best one and take it over
            import copy
            e1 = copy.deepcopy(pop.individuals[0])
            new_pop.individuals.append(e1)
            # get the Elites and optimize it with numerical method
            for t in range(self.num_elites-1):
                parent = copy.deepcopy(pop.individuals[t+1])
                parent.set_chromo(self.mutate(parent))
               
                #Optimise the local minima
                parent.set_chromo(self.local_optimiser(parent.get_chromo()))

                if len(new_pop.individuals) > 0:
                    if not np.array_equal(parent.get_chromo(), new_pop.individuals[0].get_chromo()):
                        logger.debug ( "Elite: " + str(parent.get_chromo()))
                        new_pop.individuals.append(parent)
                    else:
                        logger.debug ("First Elite: " + str(parent.get_chromo()))
                        new_pop.individuals.append(parent)

            # Crossover the population, select by tournament selection
            while len(new_pop.individuals) < self.pop_size and len(pop.individuals) > 2:
                momNum = self.tournamentSelection(pop, 3)
                mom = pop.individuals[momNum].get_chromo()
                dadNum = self.tournamentSelection(pop, 3)
                dad = pop.individuals[dadNum].get_chromo()
                off = self.gen_crossover(mom, dad)
                indi = self.individual(off,self.eval_func)
                indi.set_chromo(self.mutate(indi))

                #SLSQP
                # invest in optimization dependent from current generations
                #if (random.random()*i > numGenerations*0.5):

                #!!!!
                #Erik, check if local optimiser is working here
                #if (random.random() > 0.8):
                #    indi.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
                #        off, starting_nodes_angles), method=ga_optimizer, include_orientation=include_orientation, max_iter=max_iter))
                if (random.random() > 0.8):
                    parent.set_chromo(self.local_optimiser(parent.get_chromo()))


                if (indi.fitness < pop.individuals[momNum].fitness) or (indi.fitness < pop.individuals[momNum].fitness):
                    if momNum < dadNum:
                        del pop.individuals[dadNum]
                        del pop.individuals[momNum]
                    if momNum == dadNum:
                        del pop.individuals[dadNum]
                    if dadNum < momNum:
                        del pop.individuals[momNum]
                        del pop.individuals[dadNum]

                new_pop.individuals.append(indi)

            # Fill up the rest of the population with new individuals
            while len(new_pop.individuals) < self.pop_size:

                indi = self.individual(self.init_gene(self.chrom_length,self.bounds),self.eval_func)
                #if (random.random()*i > numGenerations*0.5):
                #if True:

                #!!!!! Erik, das muss hier wieder rein, aber dann extern gelöst, bitte!
                #if (random.random() > 0.8):
                #    indi.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
                #        indi.gene, starting_nodes_angles), method=ga_optimizer, include_orientation=include_orientation, max_iter=max_iter))
                if (random.random() > 0.8):
                    indi.set_chromo(self.local_optimiser(indi.get_chromo()))

                new_pop.individuals.append(indi)

            pop = new_pop

            '''
            for indi in pop.individuals:
                indi.refresh_fitness()
            pop.individuals = sorted(
                pop.individuals, key=lambda individual: individual.fitness)
            minFitness = pop.individuals[0].fitness
            min_distance = pop.individuals[0].distance
            min_orientation_distance = pop.individuals[0].orientation_distance
            if dist_acc==None and or_acc==None:
                acc_reached=False
            elif dist_acc==None:
                acc_reached = min_orientation_distance < or_acc    
            elif or_acc==None:
                acc_reached = min_distance < dist_acc
            else:
                acc_reached = min_distance < dist_acc and min_orientation_distance < or_acc
            print ("End criteria: " + str(acc_reached) + ' ' + str(min_distance) + ' ' + str(min_orientation_distance))
            # raw_input()
            # for indi in pop.individuals:
            '''


            #For not multithreading
            for indi in pop.individuals:
                logger.debug ("genes in population: " + str(indi))
                indi.refresh_fitness()
            
            ## Changes for multithreading
            #results=pool.map(refresh_fitness,pop.individuals)
            #pool.close()
            #pool.join()
            ##

            #sort by fitness
            pop.individuals = sorted(pop.individuals, key=lambda individual: individual.fitness)

            #Check with best gene, if accuracy is reached
            acc_reached=self.accuracy_reached(pop.individuals[0].get_chromo())
        

        #full_joint_result=chain.active_to_full(pop.individuals[0].get_chromo(), starting_nodes_angles)

        min_indi=pop.individuals[0]

        #if (multiproc_call):
            #fit_factor=min_distance * (1 - orientation_weight) + min_orientation_distance * orientation_weight
            #return (fit_factor,full_joint_result)
        #    return(min_indi.fitness , min_indi.get_chromo())
        #else:
        #    return (full_joint_result)
        
        return(min_indi.fitness , min_indi.get_chromo())
