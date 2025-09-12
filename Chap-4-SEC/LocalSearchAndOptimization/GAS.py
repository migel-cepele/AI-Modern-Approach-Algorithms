#Genetic algorithm search
import bisect
import numpy as np


class GA:
    def __init__(self, initial_state, population):
        self.population = population
        self.initial_state = initial_state


    def genetic_search(self, ngen=1000, pmut=0.1, n=20):

        s = self.initial_state



    def genetic_algorithm(self, fitness, gene_pool=[0,1], f_thres = None, ngen = 1000, pmut = 0.1):
        for i in range(ngen):
            population = [self.mutate(self.recombine(*self.select(2, population, fitness)), gene_pool, pmut)
                          for i in range(len(population))]
            
            fittest_individual = self.fitness_threshhold(fitness, f_thres, population)
            if fittest_individual:
                return fittest_individual
            
        return max(population, key=fitness)

    
    def fitness_threshhold(self, fitness_fn, f_thres, population):
        if not f_thres:
            return None
        fittest_individual = max(population, key=fitness_fn)
        if fitness_fn(fittest_individual) >= f_thres:
            return fittest_individual
        
        return None


    def init_population(self, pop_no, gene_pool, state_length):
        # pop_no is number of individuals in population
        # gene pool is list of possible values for individuals
        # state length is the length of each individual

        g = len(gene_pool)
        population = []
        for i in range(pop_no):
            new_individual = [gene_pool[np.random.randint(0,g)] for j in range(state_length)]
            population.append(new_individual)

        return population

    def select(self, r, population, fitness_fn):
        fitnesses = map(fitness_fn, population)
        sampler = self.weighted_sampler(population, fitnesses)
        return [sampler() for i in range(r)]
    

    def weighted_sampler(self,seq, weights):
        
        totals = []
        for w in weights:
            totals.append(w + totals[-1] if totals else w)
        return lambda: seq[bisect.bisect(totals, np.random.uniform(0, totals[-1]))]


    def recombine(self, x, y):
        n = len(x)
        c = np.random.randint(0, n)
        return x[:c] + y[c:]


    def recombine_uniform(self, x, y):
        n = len(x)
        result = [0] * n # list with size n with all 0
        indexes = np.random.sample(range(n), n) # returns ndarray with size n

        for i in range(n):
            ix = indexes[i]
            result[ix] = x[ix] if i < n/2 else y[ix]

        return ''.join(str(r) for r in result)

    def mutate(self, x, gene_pool, pmut):
        if np.random.uniform(0,1) >= pmut:
            return x
        
        n = len(x)
        g = len(gene_pool)
        c = np.random.randint(0,n)
        r = np.random.randint(0,g)

        new_gene = gene_pool[r]
        return x[:c] + new_gene + x[c+1:]
        
        

